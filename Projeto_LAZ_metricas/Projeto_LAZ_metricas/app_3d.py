"""Bancada 3D nativa: navegação livre e pontos originais carregados por região."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Queue
import threading
import textwrap
from time import perf_counter

import matplotlib
import numpy as np
import open3d as o3d
from open3d.visualization import gui, rendering
from scipy.spatial import cKDTree

import config
from modules.cloud_index import build_index
from modules.collision_visualization import caixas_colisao
from modules.point_picking import pick_projected
from modules.interactive_worker import executar_worker


class Bancada3D:
    def __init__(self):
        self.window = gui.Application.instance.create_window('Bancada LiDAR 3D | Inspeção da nuvem original', 1440, 940)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([.018, .025, .04, 1.])
        self.window.add_child(self.scene)
        em = self.window.theme.font_size
        self.panel = gui.ScrollableVert(.35*em, gui.Margins(em, em, em, em))
        self.window.add_child(self.panel)
        self.window.set_on_layout(self._layout)
        self.window.set_on_close(self._close)
        self.scene.set_on_mouse(self._mouse)
        self.scene.set_on_key(self._key)
        self.inbox = Queue()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='lidar-disk')
        self.cancel_index = threading.Event()
        self.index = None
        self.job = None
        self.worker = None
        self.busy = False
        self.prepared = False
        self.loaded_parameters = None
        self.detail = np.empty((0, 3))
        self.detail_tree = None
        self.focus = None
        self.region_center = None
        self.region_radius = 0.
        self.bases, self.positions, self.valid = {}, {}, {}
        self.keys = {}
        self.mode = 'inspect'
        self.pick_armed = False
        self.pov = False
        self.closed = False
        self.picking = False
        self.last_tick = perf_counter()
        self.geometries = set()
        self.result = None
        self.preview_count = 0
        self.loaded_preview_limit = None
        self.collision_index = -1
        self.searching = False
        self.voxel_mesh_ready = False
        self._controls(em)
        self.banner = gui.Label('Abra a nuvem para começar.')
        self.window.add_child(self.banner)
        self.search_cancel = gui.Button('Cancelar busca')
        self.search_cancel.set_on_clicked(self._cancel_planner)
        self.search_cancel.visible = False
        self.window.add_child(self.search_cancel)
        self._buttons()
        # A bomba agenda trabalho na thread gráfica mesmo com a câmera parada.
        self.pump_stop = threading.Event()
        self.pump_pending = threading.Event()
        threading.Thread(target=self._pump, daemon=True, name='lidar-ui-pump').start()

    def _pump(self):
        while not self.pump_stop.wait(.05):
            if self.pump_pending.is_set():
                continue
            self.pump_pending.set()
            def update():
                try:
                    if not self.closed and self._tick():
                        self.window.post_redraw()
                finally:
                    self.pump_pending.clear()
            gui.Application.instance.post_to_main_thread(self.window, update)

    def _label(self, text):
        widget = gui.Label(text)
        self.panel.add_child(widget)
        return widget

    def _button(self, text, callback):
        widget = gui.Button(text)
        widget.set_on_clicked(callback)
        self.panel.add_child(widget)
        return widget

    def _number(self, title, value, lower, upper, integer=False):
        self._label(title)
        widget = gui.NumberEdit(gui.NumberEdit.INT if integer else gui.NumberEdit.DOUBLE)
        widget.set_limits(lower, upper)
        if integer:
            widget.int_value = value
        else:
            widget.double_value = value
        self.panel.add_child(widget)
        return widget

    def _controls(self, em):
        self._label('NUVEM ORIGINAL | INSPEÇÃO 3D')
        self.path = gui.TextEdit()
        self.path.text_value = str(config.LAZ_PATH)
        self.panel.add_child(self.path)
        self._button('Escolher arquivo LAS / LAZ', self._choose)
        self.open_button = self._button('Abrir / indexar nuvem', self._open)
        self.cloud_info = self._label('O índice regional é criado uma vez em disco.\nNenhum ponto é removido do arquivo original.')
        self._label('NAVEGAÇÃO')
        self._button('Mapa inteiro', self._whole)
        self._button('Vista superior da região', self._top)
        self._button('Órbita da região', self._orbit)
        self._button('Entrar em POV', self._fly)
        self.speed = self._number('Velocidade POV (m/s)', 8., .1, 100.)
        self.radius = self._number('Raio de detalhes (m)', 60., 5., 300.)
        self.radius.set_on_value_changed(lambda _: self._region(self.focus) if self.focus is not None else None)
        self.auto = gui.Checkbox('Detalhes acompanham a câmera em POV')
        self.auto.checked = True
        self.panel.add_child(self.auto)
        self.point_size = self._number('Tamanho dos pontos (pixels)', 2., 1., 8.)
        self.point_size.set_on_value_changed(self._resize_points)
        self._label('QUANTIDADE DE PONTOS VISUALIZADOS')
        self.preview_limit = self._number('Limite da prévia global (pontos)', 300_000, 1_000, 50_000_000, integer=True)
        self.region_limit = self._number('Limite de carga regional (pontos)', 2_000_000, 1_000, 50_000_000, integer=True)
        self.apply_points = self._button('Aplicar limites de visualização', self._apply_point_limits)
        self._label('Prévia maior: reabre o índice e limpa a seleção.\nRegião: carrega todos os pontos no raio.\nUse o raio para ampliar a área detalhada.')
        self.visible_info = self._label('Pontos exibidos: 0')
        self._label('REPRESENTAÇÃO DO MAPA')
        self.map_view = gui.Combobox()
        self.map_view.add_item('Pontos originais')
        self.map_view.add_item('Voxels ocupados do A*')
        self.map_view.set_on_selection_changed(lambda _, index: self._set_map_view(index))
        self.panel.add_child(self.map_view)
        self.voxel_info = self._label('Vista voxelizada disponível após preparar a grade.')
        self.detail_info = self._label('Região detalhada: nenhuma.\nUse Inspecionar região e clique na cena.')
        self._label('SELEÇÃO | BOTÃO + CLIQUE NA CENA')
        self._button('Inspecionar região', lambda: self._set_mode('inspect'))
        self.start_button = self._button('Selecionar início', lambda: self._set_mode('inicio'))
        self.end_button = self._button('Selecionar destino', lambda: self._set_mode('destino'))
        self._button('Restaurar pontos de teste', self._restore_test_points)
        self.heights, self.position_labels = {}, {}
        for name, title in [('inicio', 'Início'), ('destino', 'Destino')]:
            self.heights[name] = self._number(f'{title}: altura acima do ponto escolhido (m)', config.TEST_HEIGHT, 0., 1000.)
            self.heights[name].set_on_value_changed(lambda _, n=name: self._height_changed(n))
            self.position_labels[name] = self._label(f'{title}: não selecionado.')
        self._label('Pontos são escolhidos na região original\ncarregada, nunca na prévia distante.\nAltura relativa à superfície, não ao solo.')
        self._label('PLANEJAMENTO | RESOLUÇÃO INDEPENDENTE')
        self.sampling = self._number('Amostragem: 1 ponto a cada', config.AMOSTRAGEM, 1, 1000, integer=True)
        self.voxel = self._number('Tamanho do voxel do A* (m)', config.VOXEL_SIZE, .01, 100.)
        self._label('A visualização detalhada NÃO altera a grade.\nVoxel de 5 m não dá resolução centimétrica.\nNão há garantia de detecção de cabos.')
        self.prepare_button = self._button('Preparar grade do A*', self._prepare)
        self.plan_button = self._button('Planejar entre os pontos', self._plan)
        self._button('Cancelar planejamento / liberar grade', self._cancel_planner)
        capture_button = self._button('Captura PNG (em validação)', self._screenshot)
        capture_button.enabled = False
        self.status = self._label('Abra a nuvem para começar.')
        self.result_label = self._label('')
        self._label('COLISÕES DA SPLINE | DIAGNÓSTICO')
        self.show_collisions = gui.Checkbox('Mostrar colisões detectadas')
        self.show_collisions.checked = True
        self.show_collisions.set_on_checked(lambda _: self._draw_collisions())
        self.panel.add_child(self.show_collisions)
        self._label('Laranja: voxels ocupados atingidos.\nMagenta: amostras da spline em colisão.\nA* vermelho | spline azul.\nDiagnóstico pela validação amostrada atual.')
        self.collision_previous = self._button('Colisão anterior', lambda: self._focus_collision(-1))
        self.collision_next = self._button('Próxima colisão / aproximar', lambda: self._focus_collision(1))
        self.collision_info = self._label('Execute o planejamento para inspecionar.')
        self._label('POV: arraste para olhar; WASD move;\nQ desce, E sobe. A câmera não tem colisão.\nÓrbita: arraste e use a roda para aproximar.\nRole este painel para acessar os controles.')

    def _layout(self, context):
        rect = self.window.content_rect
        width = 0 if self.searching else min(390, int(rect.width*.34))
        self.panel.frame = gui.Rect(rect.x, rect.y, width, rect.height)
        self.scene.frame = gui.Rect(rect.x+width, rect.y, rect.width-width, rect.height)
        self.banner.frame = gui.Rect(rect.x+width+15, rect.y+10, rect.width-width-30, self.window.theme.font_size*5)
        self.search_cancel.frame = gui.Rect(rect.get_right()-180, rect.y+100, 165, 40)

    def _notice(self, text):
        print(f'[3D] {text}', flush=True)
        self.status.text = textwrap.fill(str(text), width=43)
        self.banner.text = textwrap.fill(str(text), width=90)
        self.window.set_needs_layout()

    def _buttons(self):
        ready = self.index is not None
        self.open_button.enabled = self.job is None and not self.busy
        for field in (self.preview_limit, self.region_limit, self.apply_points):
            field.enabled = self.job is None and not self.busy
        self.map_view.enabled = ready and not self.busy
        self.prepare_button.enabled = ready and not self.busy
        self.start_button.enabled = self.end_button.enabled = ready and not self.busy
        self.plan_button.enabled = ready and self.prepared and not self.busy and self.job is None and len(self.valid) == 2 and all(self.valid.values())
        for field in [self.path, self.sampling, self.voxel, *self.heights.values()]:
            field.enabled = not self.busy
        has_collisions = bool(self._collision_data().get('voxels_colisao'))
        self.collision_previous.enabled = self.collision_next.enabled = has_collisions and not self.busy

    def _choose(self):
        dialog = gui.FileDialog(gui.FileDialog.OPEN, 'Escolher nuvem LiDAR', self.window.theme)
        dialog.add_filter('.laz .las', 'LiDAR (.laz / .las)')
        dialog.add_filter('', 'Todos os arquivos')
        dialog.set_on_cancel(self.window.close_dialog)
        def done(path):
            self.path.text_value = path
            self.window.close_dialog()
        dialog.set_on_done(done)
        self.window.show_dialog(dialog)

    def _open(self):
        if self.job is not None or self.busy:
            return
        source = Path(self.path.text_value)
        if not source.is_file():
            self._notice('Escolha um arquivo existente.')
            return
        self._stop_worker()
        self.prepared = False
        self.index = None
        self._invalidate_voxel_view()
        self.detail = np.empty((0, 3))
        self.detail_tree = None
        self.focus = self.region_center = None
        self.bases.clear()
        self.positions.clear()
        self.valid.clear()
        self.result = None
        self.collision_index = -1
        self.collision_info.text = 'Execute o planejamento para inspecionar.'
        self.preview_count = 0
        self.visible_info.text = 'Pontos exibidos: 0'
        self.detail_info.text = 'Região detalhada: nenhuma.'
        for name in list(self.geometries):
            self._remove(name)
        for name in self.position_labels:
            self.position_labels[name].text = f'{name}: não selecionado.'
        self.cancel_index.clear()
        future = self.executor.submit(build_index, source, config.OUTPUT_DIR/'cloud_cache',
                                      preview_limit=self.preview_limit.int_value,
                                      notify=lambda text: self.inbox.put(text), cancelled=self.cancel_index)
        self.job = ('index', future, None, None)
        self._notice('Abrindo índice regional da nuvem original...')
        self._buttons()

    def _material(self):
        material = rendering.MaterialRecord()
        material.shader = 'defaultUnlit'
        material.point_size = self.point_size.double_value*self.window.scaling
        return material

    def _remove(self, name):
        if name in self.geometries:
            self.scene.scene.remove_geometry(name)
            self.geometries.remove(name)

    def _cloud(self, name, points):
        self._remove(name)
        if name == 'preview':
            self.preview_count = len(points)
        self.visible_info.text = (f'Pontos exibidos: {self.preview_count+len(self.detail):,}\n'
                                  f'Prévia: {self.preview_count:,} | região: {len(self.detail):,}')
        if not len(points):
            return
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        maximum = max(float(self.index.metadata['max_relative'][2]), 1.)
        cloud.colors = o3d.utility.Vector3dVector(matplotlib.colormaps['terrain'](np.clip(points[:, 2]/maximum, 0, 1))[:, :3])
        self.scene.scene.add_geometry(name, cloud, self._material(), False)
        self.geometries.add(name)
        self._map_visibility()

    def _invalidate_voxel_view(self):
        self._remove('voxel_map')
        self.voxel_mesh_ready = False
        self.map_view.selected_index = 0
        self.voxel_info.text = 'Vista voxelizada disponível após preparar a grade.'
        self._map_visibility()

    def _map_visibility(self):
        voxel_view = self.map_view.selected_index == 1 and self.voxel_mesh_ready
        self.visible_info.text = ('Pontos originais ocultos (vista voxelizada).' if voxel_view else
            f'Pontos exibidos: {self.preview_count+len(self.detail):,}\n'
            f'Prévia: {self.preview_count:,} | região: {len(self.detail):,}')
        for name in ('preview', 'detail'):
            if name in self.geometries:
                self.scene.scene.show_geometry(name, not voxel_view)
        if 'voxel_map' in self.geometries:
            self.scene.scene.show_geometry('voxel_map', voxel_view)
        self.scene.force_redraw()

    def _set_map_view(self, index):
        if self.busy:
            return
        if index == 1 and not self.prepared:
            self.map_view.selected_index = 0
            self._notice('Prepare a grade do A* antes de visualizar seus voxels.')
        elif index == 1 and not self.voxel_mesh_ready:
            self._send('voxels', {})
            self._notice('Construindo vista voxelizada da grade preparada...')
        self._map_visibility()

    def _voxel_geometry(self, data):
        self._remove('voxel_map')
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(data['vertices'])
        mesh.triangles = o3d.utility.Vector3iVector(data['triangles'])
        maximum = max(float(self.index.metadata['max_relative'][2]), 1.)
        mesh.vertex_colors = o3d.utility.Vector3dVector(matplotlib.colormaps['terrain'](
            np.clip(data['vertices'][:, 2]/maximum, 0, 1))[:, :3])
        material = rendering.MaterialRecord()
        material.shader = 'defaultUnlit'
        self.scene.scene.add_geometry('voxel_map', mesh, material, False)
        self.geometries.add('voxel_map')
        self.voxel_mesh_ready = True
        self.voxel_info.text = f"{data['total']:,} voxels ocupados | aresta: {data['tamanho']:g} m\nInclui o subsolo bloqueado."
        self._map_visibility()
        self.window.set_needs_layout()

    def _resize_points(self, _):
        for name in ('preview', 'detail'):
            if name in self.geometries:
                self.scene.scene.modify_geometry_material(name, self._material())

    def _apply_point_limits(self):
        if self.job is not None or self.busy:
            return
        if self.index is None:
            self._notice('Limites definidos. Abra a nuvem para aplicá-los.')
        elif self.preview_limit.int_value != self.loaded_preview_limit:
            self._open()
        else:
            center = self.region_center if self.region_center is not None else self.focus
            if center is not None:
                self._region(center)
            else:
                self._notice('Limite regional aplicado. Inspecione uma região para carregar os detalhes.')

    def _set_mode(self, mode):
        self.mode = mode
        self.pick_armed = True
        self.keys.clear()
        text = 'Clique na cena para carregar a região.' if mode == 'inspect' else f'Clique em um ponto ORIGINAL carregado para definir {mode}.'
        self._notice(text)

    def _whole(self):
        if self.index is None:
            return
        self.pov = False
        self.keys.clear()
        maximum = np.asarray(self.index.metadata['max_relative'])
        center = maximum*.5
        bounds = o3d.geometry.AxisAlignedBoundingBox(np.zeros(3), np.maximum(maximum, 1.))
        self.scene.setup_camera(60., bounds, center)
        self.scene.look_at(center, center+[0, 0, max(maximum[:2])*1.15], [0, 1, 0])
        self.scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

    def _top(self):
        if self.focus is None:
            self._whole()
            return
        self.pov = False
        self.keys.clear()
        self.scene.look_at(self.focus, self.focus+[0, 0, self.radius.double_value*2.8], [0, 1, 0])
        self.scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

    def _orbit(self):
        if self.focus is None:
            self._whole()
            return
        self.pov = False
        self.keys.clear()
        radius = self.radius.double_value
        self.scene.look_at(self.focus, self.focus+[radius, -radius, radius*.8], [0, 0, 1])
        self.scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

    def _fly(self):
        if self.index is None:
            return
        if self.focus is None:
            preview = self.index.preview
            middle = np.asarray(self.index.metadata['max_relative'])[:2]*.5
            self.focus = preview[np.argmin(np.sum((preview[:, :2]-middle)**2, axis=1))].copy()
        self.keys.clear()
        eye = self.focus+[0, -min(self.radius.double_value*.4, 25.), 6.]
        self.scene.look_at(self.focus+[0, 0, 3.], eye, [0, 0, 1])
        self.scene.set_view_controls(gui.SceneWidget.Controls.FLY)
        self.pov = True
        self._notice('POV: arraste para olhar; WASD move; Q/E controla altura. Clique na cena para focar o teclado.')
        self._region(eye)

    def _region(self, center, zoom=False):
        if self.index is None or center is None or self.job is not None:
            return
        center = np.asarray(center).copy()
        radius = self.radius.double_value
        future = self.executor.submit(self.index.region, center, radius,
                                      max_points=self.region_limit.int_value)
        self.job = ('region_zoom' if zoom else 'region', future, center, radius)
        self._notice('Carregando pontos originais da região em disco...')
        self._buttons()

    def _mouse(self, event):
        if self.searching:
            return gui.Widget.EventCallbackResult.HANDLED
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN:
            self.keys.clear()
        if event.type != gui.MouseEvent.Type.BUTTON_DOWN or not (self.pick_armed or event.is_modifier_down(gui.KeyModifier.CTRL)):
            return gui.Widget.EventCallbackResult.IGNORED
        if self.map_view.selected_index == 1:
            self._notice('Alterne para Pontos originais para inspecionar ou selecionar posições na nuvem.')
            return gui.Widget.EventCallbackResult.HANDLED
        if self.index is None or self.picking or self.busy:
            return gui.Widget.EventCallbackResult.HANDLED
        frame = self.scene.frame
        x, y = int(event.x-frame.x), int(event.y-frame.y)
        width, height = int(frame.width), int(frame.height)
        model = np.asarray(self.scene.scene.camera.get_model_matrix()).copy()
        if self.job is not None:
            return gui.Widget.EventCallbackResult.HANDLED
        points = self.index.preview if self.mode == 'inspect' else self.detail
        if not len(points):
            self._notice('Carregue uma região original antes de selecionar os pontos.')
            return gui.Widget.EventCallbackResult.HANDLED
        self.picking = True
        self.pick_armed = False
        camera = self.scene.scene.camera
        future = self.executor.submit(pick_projected, points,
                                      np.asarray(camera.get_projection_matrix()).copy(),
                                      np.asarray(camera.get_view_matrix()).copy(),
                                      width, height, x, y)
        self.job = ('pick', future, (self.mode, model), None)
        self._buttons()
        return gui.Widget.EventCallbackResult.HANDLED

    def _pick(self, world):
        if self.mode == 'inspect':
            self.focus = world.copy()
            self._region(world, zoom=True)
            return
        if self.detail_tree is None:
            self._notice('Primeiro use Inspecionar região. A seleção precisa dos pontos originais.')
            return
        distance, index = self.detail_tree.query(world)
        if distance > .5:
            self._notice('O clique não corresponde a um ponto original carregado. Inspecione e aproxime essa região.')
            return
        self.bases[self.mode] = self.detail[index].copy()
        self._height_changed(self.mode)

    def _height_changed(self, name):
        if name not in self.bases:
            return
        self.positions[name] = self.bases[name]+[0, 0, self.heights[name].double_value]
        self.valid.clear()
        self._clear_paths()
        self._markers()
        self._position_texts()
        if self.prepared and not self.busy:
            self._send('validar', self.positions)
        self._buttons()

    def _restore_test_points(self):
        if self.busy or self.job is not None:
            return
        if self.index is None or Path(self.index.metadata['source']).resolve() != config.TEST_MAP_PATH.resolve():
            self._notice('Abra o mapa reboucas_alves_guimaraes_2017.laz para restaurar os pontos de teste.')
            return
        origin = np.asarray(self.index.metadata['origin'])
        # As posições salvas já incluem a altura de 5 m mostrada na imagem.
        for name, original in [('inicio', config.TEST_START_ORIGINAL), ('destino', config.TEST_GOAL_ORIGINAL)]:
            point = np.asarray(original)-origin
            self.bases[name] = point-[0., 0., config.TEST_HEIGHT]
            self.heights[name].double_value = config.TEST_HEIGHT
            self.positions[name] = point
        self.valid.clear()
        self._clear_paths()
        self._markers()
        self._position_texts()
        if self.prepared:
            self._send('validar', self.positions)
        self._buttons()
        self._notice('Pontos de teste restaurados. Prepare a grade para verificar a ocupação.')

    def _position_texts(self, states=None):
        for name, point in self.positions.items():
            original = point+np.asarray(self.index.metadata['origin'])
            state = states.get(name, {}).get('motivo', '') if states else 'Prepare a grade para validar.'
            self.position_labels[name].text = (f'{name}: XYZ relativo (m)\n'
                f'{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}\n'
                f'XYZ original: {original[0]:.3f},\n{original[1]:.3f}, {original[2]:.3f}\n{state}')
        self.window.set_needs_layout()

    def _markers(self):
        for name, point in self.positions.items():
            self._remove(name)
            # Um marcador de escala fixa em metros, sem modificar o ponto selecionado.
            marker = o3d.geometry.TriangleMesh.create_sphere(radius=.35)
            marker.translate(point)
            marker.compute_vertex_normals()
            marker.paint_uniform_color([.1, 1., .3] if name == 'inicio' else [1., .15, .25])
            material = rendering.MaterialRecord()
            material.shader = 'defaultUnlit'
            self.scene.scene.add_geometry(name, marker, material)
            self.geometries.add(name)

    def _key(self, event):
        if self.searching:
            return gui.Widget.EventCallbackResult.HANDLED
        keys = [gui.KeyName.W, gui.KeyName.A, gui.KeyName.S, gui.KeyName.D, gui.KeyName.Q, gui.KeyName.E]
        if not self.pov or event.key not in keys:
            return gui.Widget.EventCallbackResult.IGNORED
        if event.type == gui.KeyEvent.Type.DOWN:
            self.keys[event.key] = perf_counter()
        else:
            self.keys.pop(event.key, None)
        return gui.Widget.EventCallbackResult.HANDLED

    def _move(self, now, delta):
        if not self.pov or not self.keys:
            return False
        self.keys = {key: instant for key, instant in self.keys.items() if now-instant < .75}
        model = np.asarray(self.scene.scene.camera.get_model_matrix())
        right, forward, eye = model[:3, 0], -model[:3, 2], model[:3, 3]
        vectors = {gui.KeyName.W: forward, gui.KeyName.S: -forward,
                   gui.KeyName.D: right, gui.KeyName.A: -right,
                   gui.KeyName.E: np.array([0., 0., 1.]), gui.KeyName.Q: np.array([0., 0., -1.])}
        direction = sum((vectors[key] for key in self.keys), np.zeros(3))
        norm = np.linalg.norm(direction)
        if norm == 0:
            return False
        eye = eye+direction/norm*self.speed.double_value*delta
        self.scene.scene.camera.look_at(eye+forward*10, eye, [0, 0, 1])
        return True

    def _parameters(self):
        return (self.sampling.int_value, self.voxel.double_value)

    def _source_current(self):
        if self.index is None:
            return False
        source = Path(self.path.text_value).resolve()
        try:
            stat = source.stat()
        except OSError:
            return False
        meta = self.index.metadata
        return (source == Path(meta['source']) and stat.st_size == meta['source_size']
                and stat.st_mtime_ns == meta['source_mtime_ns'])

    def _prepare(self):
        if self.index is None or self.busy:
            return
        if not self._source_current():
            self._notice('O arquivo mudou. Abra/indexe novamente a nuvem antes de preparar a grade.')
            return
        self._invalidate_voxel_view()
        self.prepared = False
        self.valid.clear()
        self.loaded_parameters = self._parameters()
        self._send('carregar', dict(arquivo=self.index.metadata['source'], amostragem=self.sampling.int_value,
                                   voxel_size=self.voxel.double_value, origem_normalizacao=self.index.metadata['origin'],
                                   recorte=config.RECORTE if config.USAR_RECORTE else None))
        self._notice('Preparando grade do A*. A nuvem de inspeção permanece original.')

    def _send(self, command, data):
        if self.worker is None or not self.worker.is_alive():
            context = mp.get_context('spawn')
            self.commands, self.events = context.Queue(), context.Queue()
            self.search_events = context.Queue(maxsize=2)
            self.worker = context.Process(target=executar_worker, args=(self.commands, self.events, self.search_events), daemon=True)
            self.worker.start()
        self.busy = True
        self.commands.put((command, data))
        self._buttons()

    def _plan(self):
        if not self.prepared or self.busy or self.job is not None or len(self.positions) < 2:
            return
        if not self._source_current():
            self._notice('O arquivo mudou. Abra/indexe novamente a nuvem antes de planejar.')
            return
        if self._parameters() != self.loaded_parameters:
            self._notice('Os parâmetros mudaram. Prepare a grade novamente antes de planejar.')
            return
        self._clear_paths()
        self._begin_search()
        self._send('planejar', dict(inicio=self.positions['inicio'], destino=self.positions['destino'],
                                   output_dir=config.OUTPUT_DIR, total_time=config.FUZZY_TOTAL_TIME))

    def _begin_search(self):
        if self.worker is not None:
            try:
                while True:
                    self.search_events.get_nowait()
            except Empty:
                pass
        self.searching = True
        self.pov = False
        self.pick_armed = False
        self.keys.clear()
        self.search_camera = np.asarray(self.scene.scene.camera.get_model_matrix()).copy()
        self.search_voxel = self.loaded_parameters[1]
        self.panel.visible = False
        self.search_cancel.visible = True
        start, goal = self.positions['inicio'], self.positions['destino']
        center = (start+goal)*.5
        extent = max(float(np.linalg.norm(goal-start)), 40.)
        bounds = o3d.geometry.AxisAlignedBoundingBox(
            np.minimum(start, goal)-extent*.2, np.maximum(start, goal)+extent*.2)
        self.scene.setup_camera(60., bounds, center)
        self.scene.look_at(center, center+[0, -extent*.65, extent*1.15], [0, 0, 1])
        self._notice('A* em execução | caminho provisório em laranja | câmera fixa')

    def _end_search(self):
        if not self.searching:
            return
        self.searching = False
        self.panel.visible = True
        self.search_cancel.visible = False
        self._remove('search_path')
        self._remove('search_current')
        model = self.search_camera
        eye = model[:3, 3]
        self.scene.look_at(eye-model[:3, 2]*10., eye, model[:3, 1])
        self.scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
        self.window.set_needs_layout()

    def _search_frame(self, frame):
        points = (np.asarray(frame['caminho']).reshape(-1, 3)+.5)*self.search_voxel
        self._remove('search_path')
        if len(points) > 1:
            path = o3d.geometry.LineSet()
            path.points = o3d.utility.Vector3dVector(points)
            path.lines = o3d.utility.Vector2iVector(np.column_stack((np.arange(len(points)-1), np.arange(1, len(points)))))
            path.paint_uniform_color([1., .45, 0.])
            material = rendering.MaterialRecord()
            material.shader, material.line_width = 'unlitLine', 5.
            self.scene.scene.add_geometry('search_path', path, material, False)
            self.geometries.add('search_path')
        current = (np.asarray(frame['atual'])+.5)*self.search_voxel
        self._remove('search_current')
        marker = o3d.geometry.TriangleMesh.create_sphere(radius=max(.3, self.search_voxel*.4))
        marker.translate(current)
        marker.paint_uniform_color([1., .85, .1])
        material = rendering.MaterialRecord()
        material.shader = 'defaultUnlit'
        self.scene.scene.add_geometry('search_current', marker, material)
        self.geometries.add('search_current')
        self._notice(f"A* | {frame['visitados']:,} nós explorados | fila: {frame['fila']:,} | {frame['tempo_s']:.1f} s\n"
                     'Laranja: caminho provisório até o nó atual | amarelo: ponta da busca | câmera fixa')
        self.scene.force_redraw()

    def _clear_paths(self):
        for name in ('bruto', 'suavizado', 'collision_boxes', 'collision_samples'):
            self._remove(name)
        self.result = None
        self.collision_index = -1
        self.collision_info.text = 'Execute o planejamento para inspecionar.'
        self.result_label.text = ''

    def _collision_data(self):
        if self.result is None:
            return {}
        return self.result['metricas'].get('caminho_suavizado', {}).get('validacao_colisao', {})

    def _draw_collisions(self):
        for name in ('collision_boxes', 'collision_samples'):
            self._remove(name)
        validation = self._collision_data()
        if not self.show_collisions.checked or not validation.get('voxels_colisao'):
            self.scene.force_redraw()
            return
        size = self.result['metricas']['voxel_size_m']
        vertices, edges = caixas_colisao(validation['voxels_colisao'], size)
        boxes = o3d.geometry.LineSet()
        boxes.points = o3d.utility.Vector3dVector(vertices)
        boxes.lines = o3d.utility.Vector2iVector(edges)
        boxes.paint_uniform_color([1., .65, 0.])
        material = rendering.MaterialRecord()
        material.shader, material.line_width = 'unlitLine', 4.
        self.scene.scene.add_geometry('collision_boxes', boxes, material)
        self.geometries.add('collision_boxes')
        samples = validation.get('pontos_colisao_m', [])
        if samples:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(samples)
            cloud.paint_uniform_color([1., 0., 1.])
            material = rendering.MaterialRecord()
            material.shader, material.point_size = 'defaultUnlit', 9.*self.window.scaling
            self.scene.scene.add_geometry('collision_samples', cloud, material, False)
            self.geometries.add('collision_samples')
        self.scene.force_redraw()

    def _focus_collision(self, step):
        voxels = self._collision_data().get('voxels_colisao', [])
        if not voxels or self.busy:
            return
        self.collision_index = ((0 if step > 0 else len(voxels)-1) if self.collision_index < 0
                                else (self.collision_index+step) % len(voxels))
        voxel = np.asarray(voxels[self.collision_index])
        size = self.result['metricas']['voxel_size_m']
        self.focus = (voxel+.5)*size
        self.pov = False
        self.pick_armed = False
        self.keys.clear()
        distance = max(size*4., 8.)
        self.scene.look_at(self.focus, self.focus+[distance, -distance, distance], [0, 0, 1])
        self.scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
        original = self.focus+np.asarray(self.index.metadata['origin'])
        self.collision_info.text = (f'Voxel {self.collision_index+1} de {len(voxels)}: {tuple(voxel)}\n'
            f'Centro local (m): {self.focus[0]:.3f},\n{self.focus[1]:.3f}, {self.focus[2]:.3f}\n'
            f'Centro original (m): {original[0]:.3f},\n{original[1]:.3f}, {original[2]:.3f}\n'
            'O centro da caixa não é o ponto de contato.')
        self.window.set_needs_layout()
        self.scene.force_redraw()
        self._region(self.focus)

    def _paths(self, result):
        self._clear_paths()
        self.result = result
        for name, color in [('bruto', [1., .2, .1]), ('suavizado', [.2, .6, 1.])]:
            points = result[name]
            if points is None:
                continue
            lines = o3d.geometry.LineSet()
            lines.points = o3d.utility.Vector3dVector(points)
            lines.lines = o3d.utility.Vector2iVector([[i, i+1] for i in range(len(points)-1)])
            lines.paint_uniform_color(color)
            material = rendering.MaterialRecord()
            material.shader, material.line_width = 'unlitLine', 3.
            self.scene.scene.add_geometry(name, lines, material)
            self.geometries.add(name)
        stats = result['metricas']['astar']
        text = f"A*: {stats['tempo_planejamento_s']:.2f} s\nNós visitados: {stats['nos_visitados']:,}"
        if result['bruto'] is not None:
            safe = result['metricas']['caminho_suavizado']['validacao_colisao']['livre_de_colisao']
            text += '\nSpline: '+('sem colisão amostrada' if safe else 'COLISÃO DETECTADA • CSV não exportado')
        else:
            text += '\nNenhum caminho encontrado.'
        self.result_label.text = text
        validation = self._collision_data()
        self.collision_info.text = (f"Spline: {validation['quantidade_voxels_colisao']} voxels atingidos\n"
            f"Passo de validação: {validation['passo_validacao_m']:g} m\n"
            'Use os botões para aproximar cada voxel.' if validation else 'Nenhuma trajetória para inspecionar.')
        self._draw_collisions()
        self._notice('Planejamento concluído. Métricas em outputs/metrics/.')

    def _tick(self):
        now = perf_counter()
        moved = self._move(now, min(now-self.last_tick, .1))
        self.last_tick = now
        try:
            for _ in range(100):
                self._notice(self.inbox.get_nowait())
        except Empty:
            pass
        if self.job is not None and self.job[1].done():
            kind, future, center, radius = self.job
            self.job = None
            try:
                data = future.result()
                if kind == 'pick':
                    self.picking = False
                    mode, model = center
                    if mode != self.mode or not np.allclose(model, self.scene.scene.camera.get_model_matrix(), atol=1e-5):
                        self._notice('A câmera ou seleção mudou durante o clique. Tente novamente.')
                    elif data is None:
                        self.pick_armed = True
                        self._notice('Nenhum ponto sob o cursor. Aproxime ou aumente o tamanho dos pontos.')
                    else:
                        self._pick(data)
                elif kind == 'index':
                    self.index = data
                    self.loaded_preview_limit = self.preview_limit.int_value
                    self._cloud('preview', data.preview)
                    self.cloud_info.text = f"Original: {data.metadata['count']:,} pontos\nPrévia global: {len(data.preview):,} pontos\nDetalhes regionais sem amostragem."
                    self._whole()
                    self._notice('Nuvem pronta. Use Inspecionar região e clique na cena. Ctrl+clique repete a seleção ativa.')
                    if Path(data.metadata['source']).resolve() == config.TEST_MAP_PATH.resolve():
                        self._restore_test_points()
                else:
                    self.detail = data
                    self.detail_tree = cKDTree(data) if len(data) else None
                    self.region_center, self.region_radius = center, radius
                    self._cloud('detail', data)
                    overview = self.index.preview
                    keep = np.sum((overview-center)**2, axis=1) > radius**2
                    self._cloud('preview', overview[keep])
                    self.detail_info.text = f'Região: {len(data):,} pontos originais\nRaio: {radius:g} m | sem subamostragem'
                    self._notice('Detalhes carregados. Aproxime e selecione os pontos com Ctrl+clique.' if len(data) else 'Nenhum ponto nessa região. Inspecione uma superfície.')
                    if kind == 'region_zoom':
                        self._orbit()
            except Exception as error:
                # Não repetir automaticamente a mesma carga que excedeu o orçamento.
                self.picking = False
                if kind in ('region', 'region_zoom'):
                    self.region_center, self.region_radius = center, radius
                self._notice(str(error))
            self._buttons()
        if self.worker is not None:
            if self.searching:
                try:
                    for _ in range(2):
                        self._search_frame(self.search_events.get_nowait())
                except Empty:
                    pass
            try:
                for _ in range(20):
                    kind, data = self.events.get_nowait()
                    if kind == 'status':
                        self._notice(data)
                    elif kind == 'mapa':
                        self.prepared, self.busy = True, False
                        self._notice('Grade preparada: '+', '.join(f'{k}: {v:.1f}s' for k, v in data['tempos'].items()))
                        if self.positions:
                            self._send('validar', self.positions)
                    elif kind == 'validacao':
                        self.busy = False
                        self.valid = {name: value['valido'] for name, value in data.items()}
                        self._position_texts(data)
                    elif kind == 'voxels':
                        self.busy = False
                        self._voxel_geometry(data)
                        self._notice('Vista voxelizada pronta. Alterne a representação sem mudar a câmera.')
                    elif kind == 'resultado':
                        self.busy = False
                        self._end_search()
                        self._paths(data)
                    elif kind == 'erro':
                        self.busy = False
                        self._end_search()
                        if data.get('comando') == 'voxels':
                            self.map_view.selected_index = 0
                            self._map_visibility()
                        self._notice(data['mensagem'])
                    self._buttons()
            except Empty:
                pass
            if self.busy and not self.worker.is_alive():
                self.busy = self.prepared = False
                self._end_search()
                self._notice('O processo encerrou. Prepare a grade novamente.')
                self._buttons()
        if self.pov and self.auto.checked and self.index is not None and self.job is None:
            eye = np.asarray(self.scene.scene.camera.get_model_matrix())[:3, 3]
            if self.region_center is None or np.linalg.norm(eye-self.region_center) > self.radius.double_value*.5:
                self._region(eye)
        return moved

    def _screenshot(self):
        if self.index is None:
            self._notice('Abra uma nuvem antes de salvar a imagem.')
            return
        config.IMAGES_DIR.mkdir(parents=True, exist_ok=True)
        path = config.IMAGES_DIR/f'bancada_3d_{datetime.now():%Y%m%d_%H%M%S_%f}.png'
        try:
            # A API da Application usa a câmera da Open3DScene e não a vista padrão vazia.
            image = gui.Application.instance.render_to_image(
                self.scene.scene, int(self.scene.frame.width), int(self.scene.frame.height))
            if not np.asarray(image).size or np.ptp(np.asarray(image)) == 0:
                raise ValueError('A captura ficou vazia. Ajuste a câmera e tente novamente.')
            if not o3d.io.write_image(str(path), image):
                raise OSError('Não foi possível gravar a imagem.')
            self._notice(f'Imagem salva em outputs/images/{path.name}')
        except Exception as error:
            self._notice(str(error))

    def _stop_worker(self):
        if self.worker is not None:
            if self.worker.is_alive():
                self.worker.terminate()
            self.worker.join(timeout=2)
            for queue in (self.commands, self.events, self.search_events):
                queue.cancel_join_thread()
                queue.close()
            self.worker = None
        self.busy = False

    def _cancel_planner(self):
        self._stop_worker()
        self._end_search()
        self._invalidate_voxel_view()
        self.prepared = False
        self.valid.clear()
        self._notice('Grade liberada. A visualização continua disponível.')
        self._buttons()

    def _close(self):
        self.closed = True
        self.pump_stop.set()
        self.keys.clear()
        self.cancel_index.set()
        self._stop_worker()
        self.executor.shutdown(wait=False, cancel_futures=True)
        return True


def main():
    mp.freeze_support()
    application = gui.Application.instance
    application.initialize()
    bancada = Bancada3D()
    application.run()


if __name__ == '__main__':
    main()
