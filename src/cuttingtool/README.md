# CuttingTool


## Development summary
- Tile level (14 ~ 20) 수준으로 기존 Lanelet map을 자르는 Python tool 개발

- GUI 환경 상에서 동작하며, 크로스플랫폼을 지원하여 Windows, Mac, Linux 환경에서 동작가능


## How to Execute
```shell
foo@bar: ~/.../cuttingtool $ python cuttingtool.py
```

## Interface example

![sample-2021-03-10_14.19.37](/uploads/462517b73697c5b25fffd15d6afed946/sample-2021-03-10_14.19.37.gif)


## Structure
![cuttingtool_structure](/uploads/96407140657491010ceb6bf38ef2dd8a/cuttingtool_structure.png)
- 디렉토리는 파란색, script는 검은색이다.

- 'Input' directory는 자르고 싶은 파일을 넣어놓는 디렉토리이다. 없어도 프로그램은 구동 가능하다.  Input할 map을 찾을 때 편하도록 만들어 놓았으며, 2가지 예시의 맵이 들어있다.

- 'Output' Directory는 프로그램 실행시 자동으로 만들어지는 디렉토리이다. 프로그램이 자른 결과물들이 저장된다. 매 실행마다 'Input file name_level' 의 이름으로 세부 디렉토리를 만들어 저장한다.


## Dependency
- **cuttingtool** : python2로 실행시 python 2.7 이상, python3로 실행시 python 3.6 이상

- **권장환경** : Ubuntu 18.04


## Addition
- tk는 python 내장 라이브러리여서 기본적으로 깔려있지만, 없을시 다음 명령어로 install 할 수 있다.
    ```shell
    foo@bar: $ sudo apt-get install python-tk

    foo@bar: $ sudo apt-get install python3-tk
    ```

- RecursionError : maximum recursion depth exceeded in comparison 발생시
    ```python
    import sys
    sys.setrecursionlimit(3000)  // 숫자는 충분하게, python 에서 1000번 이상의 재귀를 막고있기에 늘려주면 된다
    ```
- Tile level 의 default 값을 바꾸고 싶으면 

    ```python
    self._level = tk.StringVar()
    levelCombo = ttk.Combobox(self._window, width=6, textvariable=self._level)
    levelCombo['values'] = ("14", "15", "16", "17", "18", "19", "20")
    levelCombo.grid(column=0, row=2)
    levelCombo.current(2)
    ```
    levelCombo.current( **value** ) 값을 바꿔주면 된다.
