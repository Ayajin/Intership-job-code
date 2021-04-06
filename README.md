# Summary

* target env: ros melodic & ubuntu 18.04 LTS

* just download and catkin_make

* **The result directory contains usage examples and results in videos and photos.**

### bicycle_model_run.sh

* How to execute :
    ```shell
    foo@bar: $ ./bicycle_model_run.sh
    ```

* run Bicycle model in Rviz. (be implemented as C++, Ros)

* execute example

![run_bicycle_model](https://user-images.githubusercontent.com/59246325/113708236-f1b4c800-971b-11eb-9fe0-1fdb8fc41066.gif)



### tmap.sh

* How to execute :
    ```shell
    foo@bar: $ ./tmap.sh
    ```

* run Tmap-API & get route. It convert path to tile ID & get neighbor tile ID

* Set destination by '2D Nav Goal'(Rviz)

* Get neighbor tile ID by 'Publish Point'(Rviz)

* Tile ID implemented as Quadratic Notation

* execute example

![run_tile_id_encoder](https://user-images.githubusercontent.com/59246325/113708371-19a42b80-971c-11eb-8715-d71dda78328e.gif)


### cuttingtool

* Cut .osm file to the desired level.

* See the cuttingtool README.md for more information
