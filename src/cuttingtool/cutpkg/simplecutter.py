# This script is example that how to inherit cutter.py

from cutpkg import cutter
import xml.etree.ElementTree as elemTree
import time

class Simplecutter(cutter.Cutter):
    '''using tkinter to build GUI environment'''
    def __init__(self, degree, doc_root):
        super().__init__(degree, doc_root)

    def cut(self):
        '''
        1. Scanning file by _scan_by_recursive func, then _tile_id_list filled
        2. Make piece by accord to _tile_id_list
        '''
        start = time.time()

        # 1. Scanning file by _scan_by_recursive func, then _tile_id_list filled
        nodes_list = self._doc_root.findall('./node')
        self._scan_by_recursive(nodes_list)

        # 2. Make piece by accord to _tile_id_list
        for tile_id in self._tile_id_list:
            result = self._make_piece_by_id(tile_id)

            if result is not None:
                tree = elemTree.ElementTree(result)
                
                filename = "".join (['./Output/', tile_id, '.osm'])
                tree.write(filename, encoding="utf-8", xml_declaration=True)

        print('Result : ' + str(round(time.time() - start, 2)) + 'sec')