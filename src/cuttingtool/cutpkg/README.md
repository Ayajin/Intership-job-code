# cutpkg


## Description
- 이 scripts에서 node의 좌표는 (x, y) 꼴의 tuple로 다뤄진다.

- 좌표들의 집합은 [(x1, y1), (x2, y2) ... ] 꼴의 tuple's list로 다뤄진다.

- 10진법 key를 사용한다.

- 변수는 `var` , 메소드는 *`method()`* 로 표시하였다. 대게 매개변수는 생략됨

- 현재 [remove](#cutter) 부분과 [check_integrity()](#cutter) 부분이 주석처리되어있다. remove 기능이 추가되면 맵이 더 빠르게 cutting되며 깔끔하게 잘리게 되지만 순서가 생기는 문제점이 있다.

## Analysis


### 목차

~~ ##### [1. encoder](#encoder) ~~

~~ ##### [2. decoder](#decoder) ~~

-> 방식 변경

##### [3. cutter](#cutter)

##### [4. gui_cutter](#gui_cutter)

### cutter

- **encoder와 decoder를 이용하여 결과물(cutting 된 맵)을 만드는 추상클래스**

- ***Input*** : `degree, doc_root(자르려고 하는 target map의 정보)`

    ***Output*** : `root (파일로 쓰기 전 완성된 Element class)`

- 다양한 cutter를 만들기 위한 추상클래스이다. 그리고 이들을 구현할 때 필요한 공통된 변수와 메소드 몇 개를 구현해 놓았다.

- 이번프로그램은 `gui_cutter` 가 상속받으며, 예제로 simplecutter.py 를 구현해놓았다.

    > 메소드별로 나누어 설명하겠다.
    > 
    > ###### *cut()*
    > 
    > - 추상메소드로서, 아래 있는 여러 메소드를 이용하여 클래스를 동작시키는 메소드이다.
    > 
    > ###### *scan_by_recursive()*
    > 
    > - map이 어떤 tile들로 이루어져 있는지 scan하는 함수이다.
    > 
    > - scan을 통해 `self._tile_id_list` 의 list를 만든다. 추후 이를 토대로 *`make_piece_by_id()`* 메소드에서 맵을 자른다.
    > 
    > - 알고리즘은 다음과 같다.
    >     1. 넘겨받은 `nodes_list` 에서 한 점을 뽑는다.
    >
    >     2. 그 점이 어떤 tile에 속해있는지 encoder로 계산한다.
    >
    >     3. Cutter class의 `tile_id_list` 리스트에 그 아이디를 추가시켜준다. (`tile_id_list`는 나중에 `make_piece_by_id()` 와 함께 `cut()`을 구현할 때 쓰인다)
    >
    >     4. `tile_id` 를 통해 tile의 범위를 decoder로 계산한다. 그 후 원래 `nodes_list`에서 그 범위에 있는 node들을 제거한 `next_list`를 만든다.
    >
    >     5. 재귀적으로 `next_list` 를 넘겨준다. 이 `next_list`가 비어있으면 break 한다.
    >
    > ###### *make_piece_by_id()*
    > 
    > - *`scan_by_recursive()`* 를 통해 만들어진 `self._tile_id_list` 를 기준으로 잘린 맵을 만드는 함수이다.
    > 
    > - 입력은 `tile_id` 를 받으며 끝에 무결성을 검사하는 함수를 사용하고 `root (Element)`를 return 한다.
    > 
    > - 알고리즘은 다음과 같다.
    >     1. 입력받은 tile_id를 통해 그 tile 에 속해있는 모든 노드들을 찾고 list를 만든다 : `base_node_id_list`
    > 
    >     2. `base_node_id_list`를 보고 이 node들을 가지고 있는 모든 way들을 찾고 list를 만든다 : `base_way_id_list`
    > 
    >     3. `base_node_id_list`를 보고 이 way들을 가지고 있는 모든 relation을 찾고 list를 만든다 : `relation_id_list`
    > *주의할 점은 relation 안에 relation이 있을 수도 있는 점을 검사해주어야 한다*
    >
    >     4. 이제 역순으로 relation이 가지고 있는 모든 way들의 id를 list로 저장한다 : `way_id_list`
    >
    >     5. 4에서 만든 list에 속한 way 들을 토대로 list들이 가지고 있는 모든 node들의 id를 list로 저장한다 : `node_id_list`
    >
    >     6. 새로운 파일을 만드는데 필요한 모든 list가 만들어졌다. 새 root에 차례로 node, way, relation을 붙여준다.
    >
    >     // 7. 새로 만든 root를 *`check_integrity()`* 로 무결성을 검사하여준다.
    >
    >     8. `root`를 return한다.
    > 
    >
    >>    - 현재 버전에서는 remove 기능을 지워져서 해당사항이 없지만, remove 기능을 살릴경우 다음 점을 참고해야한다.
    >>    - way나 relation이 tile의 크기보다 길어질 수 있는데 이럴경우 이웃 tile에 있는 모든 점들을 자신이 가져올 수 있다.
    >>    - safe code로서 `relation_id`에 멤버가 없다면 break 하는 구문을 넣었다. 이 경우, 이웃타일이 자신의 멤버들을 다 가져간 상황이다.
    >>    - remove를 할 시 지운 doc_root 를 가지고 1-6 과정을 반복해 나간다.
    >
    > ###### // check_integrity()
    >
    > - `root`의 무결성을 검사하는 메소드
    > - 속도를 위해 *`make_piece_by_id()`* 에서 `self._doc_root`에서 멤버를 지움으로 인해 생기는 문제점을 해결한다.
    >
    >     (원인은 서로 다른 way 나 relation에 중복되는 멤버들이 있기 때문이다.)
    >
    >     ![problem](/uploads/bf51c14f08b8b8036f26d6902ab0babe/problem.jpg)
    >     (문제가 되는 곳 빨간색으로 표시)
    > 
    > 
    > - 실제로 구현 결과 '멤버를 지우고 `root`를 만들어 무결성을 검사하는 것'이 '멤버를 안지우고 `root`를 만드는 것' 보다 좋은 성능을 보여주었다.
    > 
    > - 입력받은 root의 모든 node와 way, relation의 리스트를 만든다. 그 후, relation과 way 안의 존재하는 멤버들을 다 가지고 있는지 검사하고, 아닐시 `original_doc` 에서 해당 id를 가지고 있는 멤버를 붙인다.
    > 
    > - relation의 경우 relation을 멤버로 가지는 경우가 있어서 그것 또한 검사해주었다. 다행히 relation안의 relation은 더이상 relation을 멤버로 가지지 않아서 한 번만 더 검사해주어도 됬었다. 추후 이러한 관계가 깨질경우, 이 알고리즘을 재귀함수로 구현하면 될 것 같다.
    >
    > ###### *create_folder()*
    > 
    > - 단순히 입력받은 string을 토대로 디렉토리를 만들어주는 함수이다.
    > 
    > - 앞에서 언급하였듯이, Output 디렉토리를 만들어주기 위해 구현되었다.
    >


### gui_cutter

- **cutter 클래스를 상속받아 GUI 환경 상에서 프로그램을 돌리도록 만들어주는 클래스**

- Tkinter 를 사용한다. tkinter는 python 설치시 기본적으로 내장되어 있는 표준 라이브러리이며, Lightweight에 크로스 플랫폼이 지원되기 때문에 선택하게 되었다.
- 이 클래스는 생성과 동시에 실행된다. *`_run_gui_env()`* 는 tk를 이용하여 GUI 환경을 만들어주고 계속해서 사용자와 interaction하게 해준다.
- 이 메소드 안에 3가지 버튼이 있는데, 이 버튼들은
    1. Load 버튼을 눌러 `file_path`를 가져온다.
    2. Start 버튼을 눌러 *`__init__()`* 을 해주고 cut 메소드를 실행시킨다.
    3. Exit 버튼은 프로그램을 종료한다.

    로 작동한다.

- 또한 progressbar를 만들어서 현 진행상태를 볼 수 있게 해주었다.

    > cut 메소드와 progressbar 설명을 나누어서 진행하겠다.
    > 
    > ###### *cut()*
    > - progressbar부분과 GUI, 코드 실행시간을 구하는 부분은 생략, 알고리즘은 다음과 같다.
    > 1. `_doc_root`는 Load버튼에서 업데이트 되었다.
    >
    > 2. `_doc_root`를 통해 모든 노드의 리스트인 `nodes_list`를 만든다.
    >
    > 3. `nodes_list`를 `*_scan_by_recursive()*` 에 넘겨주어 `_tile_id_list`를 만든다.
    >
    > 4. `_tile_id_list`를 토대로 *`_make_piece_by_id()`* 를 호출하여 해당 타일의 `root`를 받는다. 그 후 이 `root`를 xml 문서로 쓴다.
    > 
    > ###### *progressbar*
    > 
    > - 일반버전에서는 잘라야할 타일의 갯수를 n으로 놓고, 한 타일을 완성시킬 때마다 1/n 씩 증가시켜주었다.
    >
    > > remove가 있는 버전에서는 다음과 같다. (두 번 주석 처리된 부분)
    > > - cutter클래스의 *`make_piece_by_id()`*는 이전 멤버들을 지우고 다시 실행되는 방식으로 실행되기에 점점 속도가 빨라진다. (스캔할 멤버수가 타일 하나 분량만큼 빠지기 때문)
    > > - 그렇기에 n개의 타일을 만들어야한다면 처음에는 n번, 그 다음에는 (n-1)번 ... 1번 스캔한다고 판단하였다. 그래서 가우스합으로 n(n-1)/2 의 계산으로 총 process량을 계산하였다. 
