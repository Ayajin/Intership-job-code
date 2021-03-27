from abc import abstractmethod
# from abc import ABC
from cutpkg import encoder, decoder
import xml.etree.ElementTree as elemTree
import os, copy

class Cutter(object):
    '''This class is Abstract Class'''
    def __init__(self, degree, doc_root):
        self._degree = degree
        self._doc_root = doc_root
        self._original_doc = copy.deepcopy(doc_root)
        self._tile_id_list = []

    @abstractmethod
    def cut(self):
        '''using _scan_by_recursive func and _make_piece_by_id func to customize cutting'''
        pass

    def _scan_by_recursive(self, nodes_list):
        # Input : nodes_list ( nodes_list : list of (elemTree.Element) )
        '''
        Scan osm file by this recursive func
        1. Pick any node
        2. Calculate specific tile that have first node
        3. Append specific_tile_id to tile_id_list(It use in _cut later)
        4. Make next_list of Element that not in specific tile
        5. Call recursive fucn(next_list)
        '''

        # Breaking out recursive
        if not nodes_list:
            return

        # 2. Pick any node
        pop_node = nodes_list.pop()

        # 3. Calculate specific tile that have first node
        x = float(pop_node.get('lon'))
        y = float(pop_node.get('lat'))

        tile_id = encoder.getTileID((x, y), self._degree)

        # 4. Append specific_tile_id to tile_id_list(It use in _cut later)
        self._tile_id_list.append(tile_id)

        # 5. Make next_list of Element that not in specific tile
        tile_range = decoder.decode_by_tile_id(tile_id, self._degree)
        next_list = []

        while nodes_list:
            temp = nodes_list.pop()

            lon = float(temp.get('lon'))
            lat = float(temp.get('lat'))

            if not (tile_range.lon_low <= lon and lon < tile_range.lon_high and tile_range.lat_low <= lat and lat < tile_range.lat_high):
                next_list.append(temp)

        # 6. Call recursive func(next_list)
        self._scan_by_recursive(next_list)

    def _make_piece_by_id(self, tile_id): # return root
        '''
        1. Find all node     in self._doc_root that range in specific tile(same as tile_id) 
        2. Find all way      in self._doc_root that have 1.node
        3. Find all relation in self._doc_root that have 2.way 
        4. Get all way_id   (in self._doc_root) that whole relation have
        5. Get all node_id  (in self._doc_root) that whole way have
        6. append (node_id_list), (way_id_list), (relation_id_list) to new root and remove them from self._doc_root
        7. Check integrity of root
        8. return root
        '''
        base_node_id_list, base_way_id_list, node_id_list, way_id_list, relation_id_list = [], [], [], [], []   # init

        # decoding by id
        tile_range = decoder.decode_by_tile_id(tile_id, self._degree)

        # 1. Find all node in self._doc_root that range in specific tile(same as tile_id) 
        for doc_node in self._doc_root.findall('./node'):
            lon = float(doc_node.get('lon'))
            lat = float(doc_node.get('lat'))

            if (tile_range.lon_low <= lon and lon < tile_range.lon_high and tile_range.lat_low <= lat and lat < tile_range.lat_high):
                base_node_id_list.append(doc_node.get('id'))

        # 2. Find all way in self._doc_root that have 1.node
        for doc_way in self._doc_root.findall('./way'):
            flag = False

            for nd in doc_way.iter('nd'):
                if nd.get('ref') in base_node_id_list :
                    flag = True
                    break

            if flag is True:
                base_way_id_list.append(doc_way.get('id'))

        # 3. Find all relation in self._doc_root that have 2.way
        for doc_relation in self._doc_root.findall('./relation'):
            flag = False

            for member in doc_relation.iter('member'):
                if member.get('ref') in base_way_id_list:
                    flag = True
                    break

            if flag is True:
                relation_id_list.append(doc_relation.get('id'))

                for member in doc_relation.iter('member'):
                    # if relation has member(relation), it must be added too.
                    if member.get('type') == 'relation':
                        relation_id_list.append(member.get('ref'))

        # If not exist, break
        if not relation_id_list:
            return  # return None

        # 4. Get all way_id in self._doc_root that whole relation have
        for relation in self._doc_root.findall('./relation'):
            if relation.get('id') in relation_id_list:
                for member in relation.iter('member'):
                    if member.get('type') == 'way':
                        way_id_list.append(member.get('ref'))

        # 5. Get all node_id in self._doc_root that whole way have
        for way in self._doc_root.findall('./way'):
            if way.get('id') in way_id_list:
                for nd in way.iter('nd'):
                    node_id_list.append(nd.get('ref'))

        # 6. append to new root by (node_id_list), (way_id_list), (relation_id_list) and remove them from self._doc_root
        root = elemTree.Element('osm', {"generator" :"JOSM", "version" :"0.6"})
        
        for doc_node in self._doc_root.findall('./node'):
            if doc_node.get('id') in node_id_list:
                root.append(doc_node)
                # self._doc_root.remove(doc_node)

        for doc_way in self._doc_root.findall('./way'):
            if doc_way.get('id') in way_id_list:
                root.append(doc_way)
                # self._doc_root.remove(doc_way)

        for doc_relation in self._doc_root.findall('./relation'):
            if doc_relation.get('id') in relation_id_list:
                root.append(doc_relation)
                # self._doc_root.remove(doc_relation)

        # 7. Check root integrity
        # root = self._check_integrity(root)

        # 8. return root
        return root

    # def _check_integrity(self, root):
    #     '''It get root and check integrity. If root has missing member, put it from original doc'''
    #     node_id_list, way_id_list, relation_id_list = [], [], []

    #     # 1. make id_list of node, way, relation
    #     for root_node in root.findall('./node'):
    #         node_id_list.append(root_node.get('id'))

    #     for root_way in root.findall('./way'):
    #         way_id_list.append(root_way.get('id'))

    #     for root_relation in root.findall('./relation'):
    #         relation_id_list.append(root_relation.get('id'))

    #     # 2-1. Check relation's member id(way and relation)
    #     for root_relation in root.findall('./relation'):
    #         for member in root_relation.iter('member'):
    #             mem_type = member.get('type')

    #             # If member is way
    #             if mem_type == 'way':
    #                 if member.get('ref') not in way_id_list:
    #                     missing_way = self._original_doc.find("way[@id='" + member.get('ref') + "']")
    #                     root.append(missing_way)

    #             # If member is relation
    #             elif mem_type == 'relation':
    #                 if member.get('ref') not in relation_id_list:
    #                     missing_relation = self._original_doc.find("relation[@id='" + member.get('ref') + "']")           
    #                     root.append(missing_relation)

    #                     # Check one more, because relation's relation have way too
    #                     # Fortunaly, relation's relation don't have relation anymore
    #                     for member in missing_relation.iter('member'):
    #                         if member.get('ref') not in way_id_list:
    #                             missing_way = self._original_doc.find("way[@id='" + member.get('ref') + "']")
    #                             root.append(missing_way)

    #     # 2-2. Check way's member id(node)
    #     for root_way in root.findall('./way'):
    #         for nd in root_way.iter('nd'):
    #             if nd.get('ref') not in node_id_list:
    #                 missing_node = self._original_doc.find("node[@id='" + nd.get('ref') + "']")
    #                 root.append(missing_node)

    #     return root

    def _create_folder(self, directory):
        '''Create directory'''
        try:
            if not os.path.exists(directory):
                os.makedirs(directory)
        except OSError:
            print ('Error: Creating directory. ' +  directory)