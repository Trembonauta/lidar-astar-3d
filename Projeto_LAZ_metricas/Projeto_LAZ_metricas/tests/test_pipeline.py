"""Verificações com uma nuvem sintética pequena, sem janelas ou dados privados."""
import json
import multiprocessing as mp
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import laspy
import numpy as np

from modules.pipeline import preparar_mapa, planejar, validar_ponto
from modules.interactive_worker import executar_worker


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.pasta = Path(self.temp.name)
        self.arquivo = self.pasta/'mapa.las'
        pontos = np.array([(x, y, 0.) for x in range(0, 101, 5) for y in range(0, 101, 5)] + [(100., 100., 60.)])
        header = laspy.LasHeader(point_format=3, version='1.2')
        header.offsets = np.array([500000., 7000000., 200.])
        header.scales = np.array([.01, .01, .01])
        las = laspy.LasData(header)
        las.x, las.y, las.z = (pontos + header.offsets).T
        las.write(self.arquivo)
        self.mapa = preparar_mapa(self.arquivo, amostragem=1, voxel_size=5.)

    def test_normalizacao_e_validacao(self):
        np.testing.assert_allclose(self.mapa.origem, [500000., 7000000., 200.])
        self.assertFalse(validar_ponto(self.mapa, [10., 10., 0.])['valido'])
        self.assertTrue(validar_ponto(self.mapa, [10., 10., 10.])['valido'])
        self.assertFalse(validar_ponto(self.mapa, [-1., 10., 10.])['valido'])
        self.assertFalse(validar_ponto(self.mapa, [10., 10., 100.])['valido'])
        self.assertFalse(validar_ponto(self.mapa, [np.nan, 10., 10.])['valido'])

    def test_planejamento_e_exportacao_reproduzivel(self):
        resultado = planejar(self.mapa, [10., 10., 10.], [40., 30., 10.], self.pasta/'saida')
        self.assertTrue(resultado['metricas']['astar']['encontrou_caminho'])
        self.assertTrue(Path(resultado['arquivos']['metricas']).is_file())
        self.assertTrue(Path(resultado['arquivos']['trajetoria']).is_file())
        dados = json.loads(Path(resultado['arquivos']['metricas']).read_text(encoding='utf-8'))
        self.assertEqual(dados['experimento']['inicio'], [10., 10., 10.])
        self.assertEqual(dados['experimento']['origem_normalizacao'], [500000., 7000000., 200.])

    def test_nao_exporta_spline_com_colisao(self):
        colidindo = np.array([[12.5, 12.5, 0.], [17.5, 17.5, 0.], [22.5, 22.5, 0.], [27.5, 27.5, 0.]])
        with patch('modules.pipeline.suavizar_caminho_spline', return_value=colidindo):
            resultado = planejar(self.mapa, [10., 10., 10.], [40., 30., 10.], self.pasta/'saida')
        self.assertFalse(resultado['metricas']['caminho_suavizado']['validacao_colisao']['livre_de_colisao'])
        self.assertNotIn('trajetoria', resultado['arquivos'])
        self.assertTrue(Path(resultado['arquivos']['metricas']).is_file())

    def test_rejeita_pontos_invalidos_e_mesmo_voxel(self):
        for inicio, destino in [([10., 10., 0.], [40., 30., 10.]), ([10., 10., 10.], [11., 11., 11.])]:
            with self.assertRaises(ValueError):
                planejar(self.mapa, inicio, destino)

    def test_worker_spawn_carrega_valida_e_planeja_sem_recarregar(self):
        contexto = mp.get_context('spawn')
        comandos, eventos = contexto.Queue(), contexto.Queue()
        busca = contexto.Queue(maxsize=2)
        worker = contexto.Process(target=executar_worker, args=(comandos, eventos, busca))
        worker.start()
        def aguardar(tipo):
            for _ in range(20):
                atual, dados = eventos.get(timeout=30)
                if atual == 'erro':
                    self.fail(dados['detalhes'])
                if atual == tipo:
                    return dados
            self.fail(f'Evento {tipo} não recebido')
        try:
            comandos.put(('carregar', dict(arquivo=str(self.arquivo), amostragem=1, voxel_size=5.)))
            self.assertEqual(aguardar('mapa')['total'], 442)
            comandos.put(('validar', dict(inicio=[10., 10., 10.], destino=[40., 30., 10.])))
            self.assertTrue(all(e['valido'] for e in aguardar('validacao').values()))
            comandos.put(('voxels', {}))
            mesh = aguardar('voxels')
            self.assertEqual(mesh['total'], len(self.mapa.ocupados))
            self.assertEqual(mesh['tamanho'], self.mapa.voxel_size)
            self.assertEqual(mesh['triangles'].shape[1], 3)
            comandos.put(('planejar', dict(inicio=[10., 10., 10.], destino=[40., 30., 10.])))
            self.assertTrue(aguardar('resultado')['metricas']['astar']['encontrou_caminho'])
            self.assertTrue(busca.get(timeout=5)['final'])
        finally:
            comandos.put(('fechar', None))
            worker.join(timeout=5)
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=5)
            comandos.close()
            eventos.close()
            busca.close()


if __name__ == '__main__':
    unittest.main()
