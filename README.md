# Summary

* target env: ros melodic & ubuntu 18.04 LTS

* just download and catkin_make

* The result directory contains usage examples and results in videos and photos.

### bicycle_model_run.sh

* How to execute :

    '''
    foo@bar: $ ./bicycle_model_run.sh
    '''

* run Bicycle model in Rviz. (be implemented as C++, Ros)


### tmap.sh

* How to execute :

    '''
    foo@bar: $ ./tmap.sh
    '''

* run Tmap-API & get route. It convert path to tile ID & get neighbor tile ID

* Set destination by '2D Nav Goal'(Rviz)

* Get neighbor tile ID by 'Publish Point'(Rviz)

* Tile ID implemented as Quadratic Notation


### cuttingtool

* Cut .osm file to the desired level.

* See the cuttingtool README.md for more information
