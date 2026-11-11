"""Une LAS/LAZ compatíveis sem amostrar ou transformar coordenadas."""
import argparse
from contextlib import ExitStack
from copy import deepcopy
from datetime import date
import json
from pathlib import Path

import laspy
import numpy as np


def unir_nuvens(fontes, destino):
    fontes = [Path(p).resolve() for p in fontes]
    destino = Path(destino).resolve()
    if len(fontes) < 2 or len(set(fontes)) != len(fontes):
        raise ValueError('Informe pelo menos duas fontes distintas.')
    if destino.exists() or destino.with_suffix('.json').exists():
        raise FileExistsError('O destino ou seu registro de origem já existe.')
    with ExitStack() as stack:
        readers = [stack.enter_context(laspy.open(p)) for p in fontes]
        base = readers[0].header
        def vlrs(header):
            # O VLR do compressor é recriado pelo escritor LAZ.
            return [(v.user_id, v.record_id, v.record_data_bytes()) for v in header.vlrs
                    if v.user_id != 'laszip encoded']
        for reader in readers[1:]:
            header = reader.header
            if (header.point_format != base.point_format or header.version != base.version
                    or not np.array_equal(header.scales, base.scales)
                    or not np.array_equal(header.offsets, base.offsets)
                    or vlrs(header) != vlrs(base)):
                raise ValueError('Cabeçalhos incompatíveis; é necessário revisar as fontes antes da união.')
        destino.parent.mkdir(parents=True, exist_ok=True)
        output_header = deepcopy(base)
        # O laspy lê LAS 1.0, mas sua escrita exige pelo menos LAS 1.1.
        if output_header.version < laspy.header.Version(1, 1):
            output_header.version = laspy.header.Version(1, 2)
        with laspy.open(destino, mode='w', header=output_header) as writer:
            for reader in readers:
                for points in reader.chunk_iterator(500_000):
                    writer.write_points(points)
        expected = sum(r.header.point_count for r in readers)
        expected_min = np.min([r.header.mins for r in readers], axis=0)
        expected_max = np.max([r.header.maxs for r in readers], axis=0)
        with laspy.open(destino) as result:
            if (result.header.point_count != expected
                    or not np.allclose(result.header.mins, expected_min, rtol=0, atol=1e-8)
                    or not np.allclose(result.header.maxs, expected_max, rtol=0, atol=1e-8)):
                raise RuntimeError('A saída não corresponde à contagem ou aos limites das fontes.')
        provenance = {
            'data_preparacao': date.today().isoformat(),
            'portal': 'https://novogeosampa.prefeitura.sp.gov.br/',
            'camada': 'Modelo Digital de Superfície - MDS 2017 - Articulação',
            'fontes': [{'arquivo': p.name, 'pontos': int(r.header.point_count)}
                       for p, r in zip(fontes, readers)],
            'saida': destino.name,
            'total_pontos': int(expected),
            'versao_las_saida': str(output_header.version),
            'limites_min': expected_min.tolist(),
            'limites_max': expected_max.tolist(),
            'operacao': 'Concatenação sem amostragem, transformação ou remoção de duplicados.',
            'referencia_espacial': 'Preservada das fontes; CRS não identificado no cabeçalho.'
                if base.parse_crs() is None else str(base.parse_crs()),
        }
        destino.with_suffix('.json').write_text(
            json.dumps(provenance, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return provenance


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--saida', required=True)
    parser.add_argument('fontes', nargs='+')
    args = parser.parse_args()
    print(json.dumps(unir_nuvens(args.fontes, args.saida), ensure_ascii=False, indent=2))
