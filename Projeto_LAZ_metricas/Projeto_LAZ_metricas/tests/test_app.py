"""Seleção por clique e edição de alturas, com os comandos interceptados."""
from types import SimpleNamespace
import tkinter as tk
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

from app import Bancada


class InterfaceTests(unittest.TestCase):
    def setUp(self):
        self.janela = tk.Tk()
        self.janela.withdraw()
        self.app = Bancada(self.janela)
        self.addCleanup(self.app.fechar)
        self.app.pontos = np.array([[0., 0., 0.], [5., 5., 5.], [10., 10., 10.]])
        self.app.arvore = cKDTree(self.app.pontos[:, :2])
        self.app.pronto = True
        self.app._desenhar()
        self.app.canvas.draw()

    def test_clique_referencia_superficie_e_altura(self):
        x, y = self.app.topo.transData.transform([5., 5.])
        evento = SimpleNamespace(inaxes=self.app.topo, button=1, xdata=5., ydata=5., x=x, y=y)
        with patch.object(self.app, '_enviar') as enviar:
            self.app._clicar(evento)
        np.testing.assert_allclose(self.app.selecionados['inicio'], [5., 5., 35.])
        self.assertEqual(self.app.modo.get(), 'destino')
        self.assertEqual(enviar.call_args.args[0], 'validar')

    def test_altura_invalida_nao_envia_busca(self):
        self.app.bases = {'inicio': np.zeros(3), 'destino': np.ones(3)}
        self.app.alturas['inicio'].set('nan')
        with patch('app.messagebox.showerror'), patch.object(self.app, '_enviar') as enviar:
            self.app._aplicar_alturas()
        enviar.assert_not_called()

    def test_parametros_modificados_exigem_recarga(self):
        self.app.parametros_carregados = self.app._parametros_tela()
        self.app.voxel.set('10')
        with patch('app.messagebox.showinfo') as aviso, patch.object(self.app, '_enviar') as enviar:
            self.app.planejar()
        aviso.assert_called_once()
        enviar.assert_not_called()


if __name__ == '__main__':
    unittest.main()
