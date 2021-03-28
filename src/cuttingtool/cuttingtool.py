from cutpkg import gui_cutter

if __name__ == "__main__":
    gui_cut = gui_cutter.GUICutter()    # run GUICutter


# # This script for run simplecutter.py
#
# from cutpkg import simplecutter
# import xml.etree.ElementTree as elemTree
#
#     doc = elemTree.parse('./Input/pangyo_lanelet_centerline_add_smoothing_ver0.osm')
#     # doc = elemTree.parse('./Input/DmcMap_Lanelet2_ver1_30.osm')
#     DEGREE = 16
#
#     simpleEx = simplecutter.Simplecutter(DEGREE, doc.getroot())
#     simpleEx.cut()