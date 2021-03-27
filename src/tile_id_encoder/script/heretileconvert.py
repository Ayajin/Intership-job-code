#!/usr/bin/env python3
import rospy
from math import floor, cos, atan, ceil

# ros messages and services
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

# for using Tmap_API
import requests
import json
import re
from json import loads

'''''''''''''''''''''''''''
######## NOTICE ########

In this script,
- coordinates are dealt with 'Tuple' (Because coordinates aren't have to change)
- coordinate's sets are dealt with 'List'
- Decimal number dealt with 'Int'
- Binary number and Quad number dealt with 'String'

'''''''''''''''''''''''''''

'''Unfortunatly, these global values are existed to publish and refresh rviz'''
markerArray = MarkerArray()
count = 0

def erase(i):
    '''for erase previous makerArray'''
    temp = Marker()
    temp.header.frame_id = "world"; temp.id = i
    temp.type =  temp.CYLINDER
    temp.action = temp.DELETE
    temp.scale.x = 1; temp.scale.y = 1; temp.scale.z = 1
    temp.color.a = 1; temp.color.r = 1; temp.color.g = 1; temp.color.b = 1
    temp.pose.orientation.w = 1
    temp.pose.position.x = 0
    temp.pose.position.y = 0
    temp.pose.position.z = 0

    markerArray.markers.append(temp)

tile_size = 360/(2**20)
def append_to_marker(point : tuple, count):
    '''for make new route map markerArray'''
    temp = Marker()
    temp.header.frame_id = "world"; temp.id = count
    temp.type =  temp.CUBE
    temp.action = temp.ADD
    global tile_size
    temp.scale.x = tile_size; temp.scale.y = tile_size; temp.scale.z = 0.00001
    temp.color.a = 1.0; temp.color.r = 1.0; temp.color.g = 0.5; temp.color.b = 0.0
    temp.pose.orientation.w = 1.0
    temp.pose.position.x = point[0] + tile_size/2
    temp.pose.position.y = point[1] + tile_size/2
    temp.pose.position.z = 0

    markerArray.markers.append(temp)

class Tmap_API:
    '''If you want another start or destnation, just change coordinate values'''
    def __init__(self, dest_x= "127.05259242188312", dest_y= "37.620244146378184"): # default value : eunghyun's home
        '''using Tmap API & erase previous map markerArray + generate new map markerArray'''
        url = 'https://apis.openapi.sk.com/tmap/routes?version=1'
        # startTime = "202102011300"    # startTime -> if don't set, set to 'right now'
        start_name = "HanyangUniv"
        start_X = "127.045375"
        start_Y = "37.557461"

        destination_name = "Destination"
        destination_X = dest_x
        destination_Y = dest_y

        headers = {
            "appkey" : "l7xx259a3da926c14ba2a01f154d9e430bc2",
            "version" : "1",
            "callback" : ""
        }

        payload = {
            "reqCoordType" : "WGS84GEO",
            "resCoordType" : "WGS84GEO",
            "startName" : "Start : " + str(start_name),
            "startX" : str(start_X) ,
            "startY" : str(start_Y) ,
            # "startTime" : startTime,
            "endName" : "Arrive : " + str(destination_name),
            "endX" : str(destination_X) ,
            "endY" : str(destination_Y) ,
            "endPoiId" : "",        # Destination's POI ID // POI : Point Of Interest
            "searchOption" : "0",   # 0: Optimization + Suggestion (default)
            "carType" : "1",        # 1 : Passenger Car
        }

        req = requests.post(url, json=payload, headers=headers)    
        jsonObj = json.loads(req.text)

        self.__result_list_ = []    # result ''List'' of coordinates(tuple)

        try:
            '''Parsing Data & Generate list of point(tuple)'''
            self.__result_list_.append((float(start_X), float(start_Y)))

            length_tot = len(jsonObj['features'])

            for i in range(0, length_tot):

                if isinstance(jsonObj['features'][i]['geometry']['coordinates'][0], float) is not True: # ['coordinates'] has 2 different value type. This code exist for Parsing

                    length_block = len(jsonObj['features'][i]['geometry']['coordinates'])

                    for j in range(0, length_block):
                        x = jsonObj['features'][i]['geometry']['coordinates'][j][0]
                        y = jsonObj['features'][i]['geometry']['coordinates'][j][1]

                        self.__result_list_.append((x, y))  # (x, y) is tuple that represent one point
                
                # else :  # overlapped data. If you check the result, you can figure out this information is 'representative point' that several point's set
                #     x = jsonObj['features'][i]['geometry']['coordinates'][0]
                #     y = jsonObj['features'][i]['geometry']['coordinates'][1]
                #     self.result.append(x, y)

        except:
        # If Destination's cooredinate is error, It has to be catched (ex : If you point in river)
            rospy.logerr("Please check the parameters. Link to meet destination conditions does not exist.")

    @property
    def result_list_(self):
        '''It return coordinates(tuples)'s list'''
        return self.__result_list_

class Sampling_point_to_point:
    '''Linear interpolation between two points : return list of point between start point ~ end point'''
    def __init__(self, start_point : float, end_point : float, degree : int):
        self.start_point_ = start_point
        self.end_point_ = end_point
        self.degree_ = degree    # Tile Level

        if (self.end_point_ != self.start_point_):
            self.dx_ = (self.end_point_[0] - self.start_point_[0]) / self.how_many()
            self.dy_ = (self.end_point_[1] - self.start_point_[1]) / self.how_many()
        else:
            self.dx_ = 0
            self.dy_ = 0

    def how_many(self):
        '''calculate distance between two points & count how many tile's side can divided(For calculate resolution'''
        num_of_frac = ((((self.end_point_[0] - self.start_point_[0]) ** 2) + ((self.end_point_[1] - self.start_point_[1]) ** 2)) ** 0.5) / (360 / pow(2, self.degree_))
        return ceil(num_of_frac)

    def sampling(self):
        '''return list of point between start point ~ end point'''
        result = []

        sample = list(self.start_point_)

        for _ in range(0, self.how_many() - 1): # For eliminate cumulative error, don't put last value
            sample[0] = sample[0] + self.dx_
            sample[1] = sample[1] + self.dy_
            sam = sample[0], sample[1]
            result.append(sam)

        result.append(self.end_point_)   # For eliminate cumulative error, put real value to last value
        
        return result

class Fill_gap:
    '''It get list, and return list that fill blank with Class "Sampling_point_to_point" by using Linear Interpolation'''
    def __init__(self, input_result : list):
        self.input_result_ = input_result
        self.output_result_ = [input_result[0]]
    
    def smoothing(self):
        for point in self.input_result_[1:len(self.input_result_)]:
            temp = Sampling_point_to_point(self.output_result_[len(self.output_result_) - 1], point, 20)  # last value is 'Resolution' : 20(equal to tile level)
            
            for samplepoint in temp.sampling():
                self.output_result_.append(samplepoint)
        
        return self.output_result_

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
        self.__rounded_x_ = floor( (self.longitude_ + 180) / (360 / pow(2, self.degree_)) )
        self.__rounded_y_ = floor( (self.latitude_ + 90) / (360 / pow(2, self.degree_)) )

        self.interleaving_32_to_64(self.__rounded_x_, self.__rounded_y_) # It initialize __interleaved_num_10_ & __interleaved_num_2_
        self.__tile_quadkey_4_ = self.decimal_to_quad_numeral_system(self.__interleaved_num_10_)

        self.__final_key_ = self.__interleaved_num_10_ + pow(4, self.degree_)

    def interleaving_32_to_64(self, x, y):
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
        
        return result
    
    def get_tile_pos(self) -> tuple: # Decoding

        x = self.__rounded_x_ * (360 / pow(2, self.degree_)) - 180
        y = self.__rounded_y_ * (360 / pow(2, self.degree_)) - 90
        return (x, y)

        # #It's same code : But I want to show you how decoding the interleaved_num
        # x = 0; y = 0

        # dec = int(self.__interleaved_num_10_)

        # for i in range(0, 32):
        #     x += (dec & (1 << (i * 2))) >> i
        #     y += (dec & (1 << (i * 2 + 1))) >> i + 1

        # tile_x = x * (360 / pow(2, self.degree_)) - 180
        # tile_y = y * (360 / pow(2, self.degree_)) - 90

        # return (tile_x, tile_y)

    @property
    def tile_quadkey_4_(self) -> str:
        return self.__tile_quadkey_4_

    @property
    def final_key_(self) -> int:
        return self.__final_key_

class QuadKeyToCoorPos:
    '''for Debug : Calculate Tile coordinate by QuadKey'''
    def __init__(self, num : str):
        self.__quad_key_10_ = self.quad_to_decimal(num)

    def quad_to_decimal(self, number : str) -> int:  # number : Quad-notation number(String)
        result = 0
        num = str(number)   # Safe Code for if input is integer
        length = len(num)
        
        for i in range(0, length):
            result += int(num[i]) * pow(4, length - (i + 1))
        
        return result

    def tile_coor_pos(self):
        x = 0; y = 0

        dec = int(self.__quad_key_10_)

        for i in range(0, 32):
            x += (dec & (1 << (i * 2))) >> i
            y += (dec & (1 << (i * 2 + 1))) >> i + 1

        tile_x = x * (360 / pow(2, 20)) - 180
        tile_y = y * (360 / pow(2, 20)) - 90

        return (tile_x, tile_y)
        
        
class HeretileID_List:
    '''It is Bucket(List) of tileID'''
    def __init__(self):
        self.__tile_idlist_ = ['0',]    # idlist[0] indicate total num of id
        self.__num_of_id_ = 0

    def append(self, id):
        '''don't append same(repetited) id when generated during Sampling(smoothing)'''
        if(self.__tile_idlist_[self.__num_of_id_] != id):
            self.__tile_idlist_.append(id)
            self.__num_of_id_ += 1
            self.__tile_idlist_[0] = str(self.__num_of_id_)
    
    @property
    def tile_id_list(self):
        return self.__tile_idlist_

    @property
    def num_of_id(self):
        return self.__num_of_id_

class ExpandTile:
    '''Make List : neighbor of Tile ID(Center). It Implemented by Binary Notation'''

    '''
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
        n = ExpandTile.NOTATION[r]

        if q:
            return self.decimal_to_quad(q) + n   # quad_notation
        return n

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
        '''Calculate neighbor Tile by Center_ID(#), and make List of its'''

        '''
        8 1 2
        7 # 3
        6 5 4
        '''
        result_list = [self.__center_ID_, ]
        north_id = self.__north_id_                             # 1
        north_east_id = self.calculateEast(self.__north_id_)    # 2
        east_id = self.__east_id_                               # 3
        south_east_id = self.calculateEast(self.__south_id_)    # 4
        south_id = self.__south_id_                             # 5
        south_west_id = self.calculateWest(self.__south_id_)    # 6
        west_id = self.__west_id_                               # 7
        north_west_id = self.calculateWest(self.__north_id_)    # 8

        neighbor_list = [north_id, east_id, south_id, west_id, north_east_id, south_east_id, north_west_id, south_west_id]
        
        return result_list + neighbor_list

    @property
    def result_list_(self):
        return self.__result_list_

def callback(msg):
    '''callback function for listening rviz data(2D Nav Goal)'''

    # using API to get coordinates's set
    using_api = Tmap_API(msg.pose.position.x, msg.pose.position.y)
    rough_coordinate_set = using_api.result_list_

    # fill a gap in rough data set(rough_coordinate_set)
    fill_gap = Fill_gap(rough_coordinate_set)
    result_coordinate_set = fill_gap.smoothing()

    encoder = HeretileEncoder(20)   # Set a Encoder to level 20

    ### erase previous result in Rviz
    global count
    for i in range(0, count):
        erase(i)
    count = 0
    ### erase previous result in Rviz

    htile_id_list = HeretileID_List()   # For get Heretile_ID

    for one_point in result_coordinate_set:

        encoder.inverting(one_point)   # It receive x, y value in real world

        # ### Simple route
        # tile_position = encoder.get_tile_pos()          # It return tile position that contain (x, y)
        # append_to_marker(tile_position, count)

        # htile_id_list.append(encoder.tile_quadkey_4_)   # For get Heretile_ID
        # count += 1
        # ### Simple route

        ### Expand route
        expand_tile = ExpandTile(encoder.tile_quadkey_4_)
        expand_list = expand_tile.result_list_

        for point_id in expand_list:

            htile_id_list.append(point_id)   # put tile id

            quad_to_coor = QuadKeyToCoorPos(point_id)
            tile_position = quad_to_coor.tile_coor_pos()

            append_to_marker(tile_position, count)
            count += 1
        ### Expand route

    # publish result to rviz
    global pub
    pub.publish(markerArray)
    rospy.loginfo("Destination_Longitude : " + str(msg.pose.position.x) + "/ Destination_Latitude : " + str(msg.pose.position.y))   # print out destination coordinate

    # # publish tile_id_list to simplelistener
    # tilepub = rospy.Publisher("tile_id", String, queue_size=10)
    # for point in htile_id_list.tile_id_list:
    #     tilepub.publish(point)

if __name__=='__main__':
    rospy.init_node('marker_publisher')

    pub = rospy.Publisher("t_map", MarkerArray, queue_size=10)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)

    rospy.spin()