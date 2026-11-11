"""Processo persistente: mantém a grade em memória entre planejamentos."""
import traceback
from queue import Full

import numpy as np

from modules.pipeline import preparar_mapa, planejar, validar_ponto
from modules.voxel_visualization import superficie_voxels


def executar_worker(comandos, eventos, busca=None):
    mapa = None
    informar = lambda texto: eventos.put(('status', texto))
    def informar_busca(data):
        # A busca não espera a GPU: descartar quadros quando a interface atrasa.
        try:
            busca.put_nowait(data)
        except Full:
            pass
    while True:
        comando, dados = comandos.get()
        if comando == 'fechar':
            return
        try:
            if comando == 'carregar':
                mapa = None
                mapa = preparar_mapa(**dados, informar=informar)
                # Somente a prévia vai à interface; o planejamento usa toda a nuvem processada.
                indices = np.linspace(0, len(mapa.pontos)-1, min(40000, len(mapa.pontos)), dtype=int)
                eventos.put(('mapa', dict(pontos=mapa.pontos[indices], tempos=mapa.tempos,
                                         total=len(mapa.pontos), origem=mapa.origem)))
            elif comando == 'validar':
                if mapa is None:
                    raise ValueError('Carregue um mapa primeiro.')
                eventos.put(('validacao', {nome: validar_ponto(mapa, ponto)
                                           for nome, ponto in dados.items()}))
            elif comando == 'voxels':
                if mapa is None:
                    raise ValueError('Prepare a grade primeiro.')
                informar('Construindo a superfície dos voxels ocupados...')
                vertices, triangles = superficie_voxels(mapa.ocupados, mapa.voxel_size)
                eventos.put(('voxels', dict(vertices=vertices, triangles=triangles,
                                            total=len(mapa.ocupados), tamanho=mapa.voxel_size)))
            elif comando == 'planejar':
                if mapa is None:
                    raise ValueError('Carregue um mapa primeiro.')
                resultado = planejar(mapa, **dados, informar=informar,
                                      informar_busca=informar_busca if busca is not None else None)
                eventos.put(('resultado', resultado))
            else:
                raise ValueError(f'Comando desconhecido: {comando}')
        except Exception as erro:
            eventos.put(('erro', dict(comando=comando, mensagem=str(erro),
                                     detalhes=traceback.format_exc())))
