#!/usr/bin/env python3
import rospy
from math import floor

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

class HeretileEncoder:
    '''change to Heretile ID, It act like Calculator'''
    NOTATION = '01234'    # For Converting

    def __init__(self, degree : int):
        self.degree_ = degree
        self.longitude_ = 0.0; self.latitude_ = 0.0

        self.__rounded_x_ = 0; self.__rounded_y_ = 0
        self.__interleaved_num_2_ = ''
        self.__interleaved_num_10_ = 0

        self.__tile_quadkey_4_ = ''
        self.__final_key_ = ''

    def inverting(self, coordinate : tuple):
        self.longitude_ = coordinate[0]
        self.latitude_ = coordinate[1]
        self.__rounded_x_ = floor( (self.longitude_ + 180) / (360 / (2 ** self.degree_) ))
        self.__rounded_y_ = floor( (self.latitude_ + 90) / (360 / (2 ** self.degree_) ))

        self.interleaving_32_to_64(self.__rounded_x_, self.__rounded_y_) # It initialize __interleaved_num_10_ & __interleaved_num_2_
        self.__tile_quadkey_4_ = self.decimal_to_quad_numeral_system(self.__interleaved_num_10_)

        self.__final_key_ = self.__interleaved_num_10_ + pow(4, self.degree_)

    def interleaving_32_to_64(self, x : int, y : int):
        result_x = 0
        result_y = 0

        for i in range(0, 32):
            result_x += (x & (1 << i)) << i
            result_y += (y & (1 << i)) << (i + 1)
        
        self.__interleaved_num_10_ = result_x + result_y
        self.__interleaved_num_2_ = self.decimal_to_binary_numeral_system(self.__interleaved_num_10_)

    def decimal_to_binary_numeral_system(self, number : int) -> str: # number : Int, return String of Binary_notation number
        q, r = divmod(number, 2)
        n = HeretileEncoder.NOTATION[r]

        if q:
            return self.decimal_to_binary_numeral_system(q) + n
        return n
    
    def decimal_to_quad_numeral_system(self, number : int) -> str:   # number : Int, return String of Quad_notation number
        q, r = divmod(number, 4)
        n = HeretileEncoder.NOTATION[r]

        if q:
            return self.decimal_to_quad_numeral_system(q) + n
        return n

    def quad_to_decimal(self, number : str) -> int:  # number : Quad-notation number(String)
        result = 0
        num = str(number)   # Safe Code for if input is integer
        length = len(num)
        
        for i in range(0, length):
            result += int(num[i]) * pow(4, length - (i + 1))
        
        return result   # It return decimal number (integer)
    
    def get_tile_pos(self) -> tuple: # Decoding
        x = self.__rounded_x_ * (360 / pow(2, self.degree_)) - 180
        y = self.__rounded_y_ * (360 / pow(2, self.degree_)) - 90
        return (x, y)

    @property
    def tile_quadkey_4_(self) -> str:
        return self.__tile_quadkey_4_

    @property
    def final_key_(self) -> int:
        return self.__final_key_

class NeighborTile:
    '''
    Make List : neighbor of Tile ID(Center). It Implemented by Binary Notation
    It operates by 'Quadratic Notation' (str)
    
    It doesn't work at end line of world map.
    But I don't catch conner case.
    Because Conditional Statement(ex : if) consume resource.
    It causes program performance degradation
    But Don't warry. End line of world map is 'ocean'. Car can't go :)
    '''
    NOTATION = '0123'    # For Converting

    def __init__(self, center_ID : str):  # Input must be quad_notation
        '''Get ID & calculate neighbor's ID'''
        self.__center_ID_ = center_ID

        self.__north_id_ = self.calculateNorth(self.__center_ID_)
        self.__east_id_ = self.calculateEast(self.__center_ID_)
        self.__south_id_ = self.calculateSouth(self.__center_ID_)
        self.__west_id_ = self.calculateWest(self.__center_ID_)

        self.__result_list_ = self.expand()

    def quad_to_decimal(self, number : str) -> int:  # number : Quad-notation number
        result = 0
        num = str(number)   # Safe Code for if input is integer
        length = len(num)
        
        for i in range(0, length):
            result += int(num[i]) * pow(4, length - (i + 1))
        
        return result   # It return integer

    def decimal_to_quad(self, number : int) -> str:  # number : Decimal number, It return Quad-notation number(string) that expressed in decimal
        q, r = divmod(number, 4)
        n = NeighborTile.NOTATION[r]

        if q:
            return self.decimal_to_quad(q) + n   # quad_notation (str)
        return n

    '''
    calculate* functions are implemented by binary puzzle in HereTile ID rule.
    If you don't understand, feel free to ask! (ayajin1018@gmail.com)
    '''
    def calculateEast(self, id : str) -> str:
        temp = self.quad_to_decimal(id)
        move = 0
        count = 0
        while (temp % 2 != 0):
            move += (0b10 << count)
            temp = temp >> 2
            count += 2

        return self.decimal_to_quad( self.quad_to_decimal(id) + (move + 1) )

    def calculateWest(self, id : str) -> str:
        temp = self.quad_to_decimal(id)
        move = 0
        count = 0
        while (temp % 2 == 0):
            move += (0b10 << count)
            temp = temp >> 2
            count += 2

        return self.decimal_to_quad( self.quad_to_decimal(id) - (move + 1) )

    def calculateNorth(self, id : str) -> str:
        temp = self.quad_to_decimal(id) >> 1
        move = 0
        count = 2
        while (temp % 2 != 0):
            move += (0b01 << count)
            temp = temp >> 2
            count += 2

        return self.decimal_to_quad( self.quad_to_decimal(id) + (move + 2) )

    def calculateSouth(self, id : str) -> str:
        temp = self.quad_to_decimal(id) >> 1
        move = 0
        count = 2
        while (temp % 2 == 0):
            move += (0b01 << count)
            temp = temp >> 2
            count += 2

        return self.decimal_to_quad( self.quad_to_decimal(id) - (move + 2) )

    def expand(self):
        '''
        Calculate neighbor Tile by Center_ID(#), and make List of its

        8 1 2
        7 # 3
        6 5 4
        '''
        result_list = [self.__center_ID_, ]
        north_id = self.__north_id_                     # 1
        east_id = self.__east_id_                       # 3
        south_id = self.__south_id_                     # 5
        west_id = self.__west_id_                       # 7
        north_east_id = self.calculateEast(north_id)    # 2
        south_east_id = self.calculateEast(south_id)    # 4
        south_west_id = self.calculateWest(south_id)    # 6
        north_west_id = self.calculateWest(north_id)    # 8

        neighbor_list = [north_id, east_id, south_id, west_id, north_east_id, south_east_id, north_west_id, south_west_id]
        
        return result_list + neighbor_list

    '''These functions are spare fuctions that if you want this class operate by Decimal Notation:int '''
    def final_key_decoder(self, final_key : int, degree : int):
        self.__center_ID_ = self.decimal_to_quad(final_key - pow(4, degree))

    def final_key_incoder(self, degree : int) -> int:
        return self.quad_to_decimal(self.__center_ID_) + pow(4, degree)
    '''These functions are spare fuctions that if you want this class operate by Decimal Notation:int '''

    @property
    def result_list_(self):
        return self.__result_list_



def callback(msg):
    x = msg.point.x
    y = msg.point.y
    coordinate = (x, y)

    encoder = HeretileEncoder(20)   # Set degree(tile level) 20
    encoder.inverting(coordinate)

    expand = NeighborTile(encoder.tile_quadkey_4_)
    need_tile_id_list = expand.result_list_

    global tilepub

    tilepub.publish("------ neighbor ------")
    for id in need_tile_id_list:
        tilepub.publish(id)
    tilepub.publish("------ neighbor ------")

if __name__ == '__main__':
    rospy.init_node('position_listener')

    tilepub = rospy.Publisher("tile_id", String, queue_size=10)
    rospy.Subscriber('/clicked_point', PointStamped, callback)

    rospy.spin()