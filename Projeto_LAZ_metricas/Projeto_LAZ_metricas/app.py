"""Bancada local: seleção de pontos, planejamento e diagnóstico visual."""
import multiprocessing as mp
from datetime import datetime
from pathlib import Path
from queue import Empty
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy.spatial import cKDTree

import config
from modules.interactive_worker import executar_worker


class Bancada:
    def __init__(self, janela):
        self.janela = janela
        janela.title('Bancada LiDAR | Planejamento A* 3D')
        janela.geometry('1320x870')
        janela.minsize(1000, 700)
        self.processo = None
        self.pronto = False
        self.ocupado = False
        self.pontos = None
        self._id_exibido = None
        self.bases = {}
        self.selecionados = {}
        self.validos = {}
        self.resultado = None
        self.parametros_carregados = None
        self.modo = tk.StringVar(value='inicio')
        self.arquivo = tk.StringVar(value=str(config.LAZ_PATH))
        self.amostragem = tk.StringVar(value=str(config.AMOSTRAGEM))
        self.voxel = tk.StringVar(value=str(config.VOXEL_SIZE))
        self.recortar = tk.BooleanVar(value=config.USAR_RECORTE)
        self.alturas = {nome: tk.StringVar(value='30') for nome in ('inicio', 'destino')}
        self.estado = tk.StringVar(value='Escolha o arquivo e clique em Carregar mapa.')
        self.rotulos = {nome: tk.StringVar(value='Nenhum ponto selecionado.\n\n') for nome in self.alturas}
        self.controles = []
        self._montar_tela()
        janela.protocol('WM_DELETE_WINDOW', self.fechar)
        janela.after(150, self._receber)

    def _montar_tela(self):
        superior = ttk.Frame(self.janela, padding=10)
        superior.pack(fill='x')
        ttk.Label(superior, text='Arquivo LiDAR').grid(row=0, column=0, sticky='w')
        entrada = ttk.Entry(superior, textvariable=self.arquivo, width=85)
        entrada.grid(row=0, column=1, columnspan=5, sticky='ew', padx=6)
        procurar = ttk.Button(superior, text='Procurar...', command=self._procurar)
        procurar.grid(row=0, column=6)
        ttk.Label(superior, text='Amostragem (1 a cada)').grid(row=1, column=0, pady=8)
        amostra = ttk.Entry(superior, textvariable=self.amostragem, width=8)
        amostra.grid(row=1, column=1, sticky='w')
        ttk.Label(superior, text='Voxel (m)').grid(row=1, column=2)
        voxel = ttk.Entry(superior, textvariable=self.voxel, width=8)
        voxel.grid(row=1, column=3, sticky='w')
        recorte = ttk.Checkbutton(superior, text='Usar recorte do config.py', variable=self.recortar)
        recorte.grid(row=1, column=4, padx=8)
        carregar = ttk.Button(superior, text='Carregar mapa', command=self.carregar)
        carregar.grid(row=1, column=5, padx=8)
        self.cancelar = ttk.Button(superior, text='Cancelar operação', command=self._cancelar)
        self.cancelar.grid(row=1, column=6)
        superior.columnconfigure(1, weight=1)
        self.controles.extend([entrada, procurar, amostra, voxel, recorte, carregar])

        centro = ttk.Frame(self.janela)
        centro.pack(fill='both', expand=True)
        painel = ttk.Frame(centro, padding=10, width=290)
        painel.pack(side='left', fill='y')
        ttk.Label(painel, text='1. Selecione no mapa superior', font=('', 11, 'bold')).pack(anchor='w')
        for nome, titulo in [('inicio', 'Início'), ('destino', 'Destino')]:
            grupo = ttk.LabelFrame(painel, text=titulo, padding=8)
            grupo.pack(fill='x', pady=6)
            radio = ttk.Radiobutton(grupo, text=f'Selecionar {titulo.lower()}', variable=self.modo, value=nome)
            radio.pack(anchor='w')
            ttk.Label(grupo, text='Altura acima da superfície (m)').pack(anchor='w')
            altura = ttk.Entry(grupo, textvariable=self.alturas[nome], width=12)
            altura.pack(anchor='w', pady=3)
            ttk.Label(grupo, textvariable=self.rotulos[nome], wraplength=255).pack(anchor='w')
            self.controles.extend([radio, altura])
        aplicar = ttk.Button(painel, text='Aplicar alturas e validar', command=self._aplicar_alturas)
        aplicar.pack(fill='x', pady=5)
        self.controles.append(aplicar)
        self.planejar_btn = ttk.Button(painel, text='2. Planejar caminho', command=self.planejar)
        self.planejar_btn.pack(fill='x', pady=5)
        ttk.Button(painel, text='Salvar imagem da bancada', command=self._salvar_imagem).pack(fill='x', pady=5)
        ttk.Label(painel, text='Clique na vista superior (esquerda).\nNa vista 3D, arraste para girar.\nDesative zoom/pan antes de selecionar.', wraplength=260).pack(anchor='w', pady=10)
        ttk.Label(painel, text='A altura é relativa ao ponto LiDAR escolhido, que pode ser telhado ou vegetação. Não é uma classificação de terreno.', wraplength=260).pack(anchor='w')
        self.diagnostico = tk.Text(painel, width=35, height=6, wrap='word', state='disabled')
        self.diagnostico.pack(fill='both', expand=True, pady=10)

        visual = ttk.Frame(centro)
        visual.pack(side='left', fill='both', expand=True)
        self.figura = Figure(figsize=(10, 6), dpi=100)
        self.topo = self.figura.add_subplot(121)
        self.cena = self.figura.add_subplot(122, projection='3d', computed_zorder=False)
        self.canvas = FigureCanvasTkAgg(self.figura, master=visual)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, visual, pack_toolbar=False)
        self.toolbar.pack(fill='x')
        self.canvas.mpl_connect('button_press_event', self._clicar)
        rodape = ttk.Frame(self.janela, padding=10)
        rodape.pack(fill='x')
        self.progresso = ttk.Progressbar(rodape, mode='indeterminate', length=150)
        self.progresso.pack(side='right')
        ttk.Label(rodape, textvariable=self.estado, wraplength=1000).pack(side='left')
        self._desenhar()
        self._atualizar_botoes()

    def _procurar(self):
        arquivo = filedialog.askopenfilename(filetypes=[('Nuvens LiDAR', '*.laz *.las'), ('Todos', '*.*')])
        if arquivo:
            self.arquivo.set(arquivo)

    def _log(self, texto):
        self.diagnostico.configure(state='normal')
        self.diagnostico.insert('end', texto + '\n\n')
        self.diagnostico.see('end')
        self.diagnostico.configure(state='disabled')

    def _atualizar_botoes(self):
        for controle in self.controles:
            controle.configure(state='disabled' if self.ocupado else 'normal')
        self.planejar_btn.configure(state='normal' if self.pronto and not self.ocupado and
                                   len(self.validos) == 2 and all(self.validos.values()) else 'disabled')
        self.cancelar.configure(state='normal' if self.ocupado else 'disabled')
        if self.ocupado:
            self.progresso.start(15)
        else:
            self.progresso.stop()

    def _enviar(self, comando, dados):
        if self.processo is None or not self.processo.is_alive():
            contexto = mp.get_context('spawn')
            self.comandos, self.eventos = contexto.Queue(), contexto.Queue()
            self.processo = contexto.Process(target=executar_worker, args=(self.comandos, self.eventos), daemon=True)
            self.processo.start()
        self.ocupado = True
        self._atualizar_botoes()
        self.comandos.put((comando, dados))

    def carregar(self):
        try:
            arquivo = Path(self.arquivo.get())
            if not arquivo.is_file():
                raise ValueError('Selecione um arquivo existente.')
            amostragem, voxel = int(self.amostragem.get()), float(self.voxel.get())
            if amostragem < 1 or not np.isfinite(voxel) or voxel <= 0:
                raise ValueError('Amostragem e voxel devem ser positivos.')
        except ValueError as erro:
            messagebox.showerror('Parâmetros', str(erro))
            return
        self.pronto = False
        self.parametros_carregados = self._parametros_tela()
        self.pontos = None
        self.bases.clear()
        self.selecionados.clear()
        self.validos.clear()
        self.resultado = None
        for rotulo in self.rotulos.values():
            rotulo.set('Nenhum ponto selecionado.\n\n')
        self._desenhar()
        self.estado.set('Carregando mapa...')
        self._enviar('carregar', dict(arquivo=str(arquivo), amostragem=amostragem,
                                      voxel_size=voxel, recorte=config.RECORTE if self.recortar.get() else None))

    def _clicar(self, evento):
        if not self.pronto or self.ocupado or evento.inaxes is not self.topo or evento.button != 1 or self.toolbar.mode:
            return
        _, indice = self.arvore.query([evento.xdata, evento.ydata])
        base = self.pontos[indice]
        pixel = self.topo.transData.transform(base[:2])
        if np.linalg.norm(pixel - [evento.x, evento.y]) > 18:
            self.estado.set('Clique próximo a um ponto visível da nuvem.')
            return
        nome = self.modo.get()
        try:
            altura = float(self.alturas[nome].get())
            if not np.isfinite(altura) or altura < 0:
                raise ValueError
        except ValueError:
            messagebox.showerror('Altura', 'Informe uma altura finita maior ou igual a zero.')
            return
        self.bases[nome] = base.copy()
        self._aplicar_alturas()
        if nome == 'inicio':
            self.modo.set('destino')

    def _aplicar_alturas(self):
        if not self.pronto or self.ocupado or not self.bases:
            return
        try:
            alturas = {nome: float(self.alturas[nome].get()) for nome in self.bases}
            if any(not np.isfinite(h) or h < 0 for h in alturas.values()):
                raise ValueError
        except ValueError:
            messagebox.showerror('Altura', 'As alturas devem ser finitas e maiores ou iguais a zero.')
            return
        self.selecionados = {nome: base + [0, 0, alturas[nome]] for nome, base in self.bases.items()}
        self.validos.clear()
        self.resultado = None
        self.estado.set('Validando na grade usada pelo A*...')
        self._desenhar()
        self._enviar('validar', self.selecionados)

    def planejar(self):
        if not self.pronto or self.ocupado:
            return
        if self._parametros_tela() != self.parametros_carregados:
            messagebox.showinfo('Mapa alterado', 'Você mudou os parâmetros do mapa. Clique em Carregar mapa antes de planejar.')
            return
        # Sempre aplicar edições de altura antes da validação final no worker.
        try:
            pontos = {}
            for nome in ('inicio', 'destino'):
                altura = float(self.alturas[nome].get())
                if not np.isfinite(altura) or altura < 0:
                    raise ValueError('Altura inválida.')
                pontos[nome] = self.bases[nome] + [0, 0, altura]
        except (ValueError, KeyError):
            messagebox.showerror('Seleção', 'Selecione os dois pontos e confira as alturas.')
            return
        if any(not np.array_equal(pontos[nome], self.selecionados.get(nome)) for nome in pontos):
            self._aplicar_alturas()
            return
        self.selecionados = pontos
        self.resultado = None
        self._desenhar()
        self.estado.set('Executando A*...')
        self._enviar('planejar', dict(inicio=pontos['inicio'], destino=pontos['destino'],
                                     output_dir=config.OUTPUT_DIR, total_time=config.FUZZY_TOTAL_TIME))

    def _parametros_tela(self):
        return (self.arquivo.get(), self.amostragem.get(), self.voxel.get(), self.recortar.get())

    def _desenhar(self):
        elev, azim = self.cena.elev, self.cena.azim
        manter_vista = self.pontos is not None and self._id_exibido == id(self.pontos)
        limites = self.topo.get_xlim(), self.topo.get_ylim()
        self.topo.clear()
        self.cena.clear()
        self.topo.set(title='Selecione início e destino', xlabel='X relativo (m)', ylabel='Y relativo (m)')
        self.topo.set_aspect('equal', adjustable='box')
        self.cena.set(title='Conferência 3D', xlabel='X (m)', ylabel='Y (m)', zlabel='Z relativo (m)')
        self.cena.view_init(elev=elev, azim=azim)
        if self.pontos is not None:
            p = self.pontos
            self.topo.scatter(p[:, 0], p[:, 1], c=p[:, 2], cmap='terrain', s=2, rasterized=True)
            self.cena.scatter(p[:, 0], p[:, 1], p[:, 2], c=p[:, 2], cmap='terrain', s=1, alpha=.5, rasterized=True)
            extensao = np.maximum(np.ptp(p, axis=0), 1.)
            self.cena.set_box_aspect(extensao)
            for nome, ponto in self.selecionados.items():
                cor = '#007d50' if nome == 'inicio' else '#ce214b'
                for eixo in [self.topo, self.cena]:
                    valores = ponto[:2] if eixo is self.topo else ponto
                    eixo.scatter(*valores, c=cor, s=85, edgecolors='white', label=nome, zorder=10)
                base = self.bases[nome]
                self.cena.plot([base[0], ponto[0]], [base[1], ponto[1]], [base[2], ponto[2]], color=cor, linestyle='--')
            if self.resultado is not None and self.resultado['bruto'] is not None:
                for chave, cor in [('bruto', '#d32f2f'), ('suavizado', '#235cff')]:
                    caminho = self.resultado[chave]
                    self.topo.plot(caminho[:, 0], caminho[:, 1], color=cor, label=chave)
                    self.cena.plot(*caminho.T, color=cor, label=chave)
            if self.selecionados:
                self.topo.legend(loc='best')
        if manter_vista:
            self.topo.set_xlim(limites[0])
            self.topo.set_ylim(limites[1])
        self._id_exibido = id(self.pontos)
        self.cena.tick_params(labelsize=8, pad=0)
        self.figura.subplots_adjust(left=.08, right=.89, wspace=.32, bottom=.12, top=.9)
        self.canvas.draw_idle()

    def _receber(self):
        try:
            if self.processo is not None:
                for _ in range(30):
                    tipo, dados = self.eventos.get_nowait()
                    if tipo == 'status':
                        self.estado.set(dados)
                    elif tipo == 'mapa':
                        self.pontos = dados['pontos']
                        self.arvore = cKDTree(self.pontos[:, :2])
                        self.pronto, self.ocupado = True, False
                        self.estado.set('Mapa pronto. Selecione um ponto na vista superior.')
                        self._log(f"Mapa: {dados['total']:,} pontos processados; {len(self.pontos):,} na prévia.\n" +
                                  '\n'.join(f'{k}: {v:.2f} s' for k, v in dados['tempos'].items()))
                        self._desenhar()
                    elif tipo == 'validacao':
                        self.ocupado = False
                        self.validos = {nome: estado['valido'] for nome, estado in dados.items()}
                        for nome, estado in dados.items():
                            p = self.selecionados[nome]
                            self.rotulos[nome].set(f'X={p[0]:.1f}  Y={p[1]:.1f}  Z={p[2]:.1f} m\nVoxel: {estado["voxel"]}\n{estado["motivo"]}')
                        self.estado.set('Seleção validada. Escolha o outro ponto ou planeje o caminho.')
                    elif tipo == 'resultado':
                        self.ocupado = False
                        self.resultado = dados
                        stats = dados['metricas']['astar']
                        self.estado.set('Planejamento concluído.' if stats['encontrou_caminho'] else 'Nenhum caminho encontrado.')
                        self._log(f"A*: {stats['tempo_planejamento_s']:.2f} s; {stats['nos_visitados']:,} nós visitados.")
                        if stats['encontrou_caminho']:
                            for nome in ('caminho_bruto', 'caminho_suavizado'):
                                metrica = dados['metricas'][nome]
                                livre = metrica['validacao_colisao']['livre_de_colisao']
                                self._log(f"{nome}: {metrica['comprimento_m']:.1f} m\nColisão por amostragem: {'não detectada' if livre else 'DETECTADA'}")
                            if 'trajetoria' not in dados['arquivos']:
                                self._log('CSV não exportado: a trajetória suavizada apresentou colisão.')
                        self._log('Arquivos:\n' + '\n'.join(dados['arquivos'].values()))
                        self._desenhar()
                    elif tipo == 'erro':
                        self.ocupado = False
                        self.estado.set(dados['mensagem'])
                        self._log(dados['detalhes'])
                        messagebox.showerror('Operação não concluída', dados['mensagem'])
                    self._atualizar_botoes()
        except Empty:
            pass
        if self.processo is not None and self.ocupado and not self.processo.is_alive():
            self.ocupado = self.pronto = False
            self.estado.set('O processo encerrou. Carregue o mapa novamente.')
            self._atualizar_botoes()
        self.janela.after(150, self._receber)

    def _salvar_imagem(self):
        if self.pontos is None:
            return
        config.IMAGES_DIR.mkdir(parents=True, exist_ok=True)
        arquivo = config.IMAGES_DIR/f'bancada_{datetime.now():%Y%m%d_%H%M%S_%f}.png'
        self.figura.savefig(arquivo, dpi=160)
        self._log(f'Imagem salva: {arquivo}')

    def _encerrar_worker(self):
        if self.processo is not None:
            if self.processo.is_alive():
                self.processo.terminate()
            self.processo.join(timeout=2)
            self.comandos.cancel_join_thread()
            self.eventos.cancel_join_thread()
            self.comandos.close()
            self.eventos.close()
            self.processo = None

    def _cancelar(self):
        self._encerrar_worker()
        self.ocupado = self.pronto = False
        self.validos.clear()
        self.estado.set('Operação cancelada. Recarregue o mapa para continuar.')
        self._atualizar_botoes()

    def fechar(self):
        self._encerrar_worker()
        self.janela.destroy()


def main():
    mp.freeze_support()
    janela = tk.Tk()
    Bancada(janela)
    janela.mainloop()


if __name__ == '__main__':
    main()
