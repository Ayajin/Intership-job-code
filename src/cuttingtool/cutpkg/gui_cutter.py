from cutpkg import cutter
import xml.etree.ElementTree as elemTree
import sys, time

# import tk
try:
    # Python 3.7
    import tkinter as tk
    import tkinter.ttk as ttk
    from tkinter import messagebox as tkMessageBox
    from tkinter import filedialog as tkFileDialog
except ImportError:
    # Python 2
    import Tkinter as tk
    import tkFileDialog, tkMessageBox
    import ttk

class GUICutter(cutter.Cutter):
    '''using tkinter to build GUI environment'''
    def __init__(self):
        self._run_gui_env()

    def _run_gui_env(self):
        '''Set up GUI_environment & Run in TK that interact with buttons'''
        self._file_path = ''

        self._window = tk.Tk()
        self._window.title("Josm Cutter")
        self._window.resizable(0, 0)

        self._progressbar = ttk.Progressbar(self._window, maximum = 100, mode = "determinate")
        self._progressbar.grid(column=1, row=2)

        self._file_dir_screen = tk.Entry(self._window, font=5)
        self._file_dir_screen.grid(column=0, row=0, columnspan=3)

        menuBar = tk.Menu(self._window)
        fileMenu = tk.Menu(menuBar, tearoff=0)
        fileMenu.add_command(label="Load", command=self.click_load)
        fileMenu.add_separator()
        fileMenu.add_command(label="Exit", command=self.click_exit)
        menuBar.add_cascade(label="File", menu=fileMenu)
        self._window.config(menu=menuBar)

        self._level = tk.StringVar()
        levelCombo = ttk.Combobox(self._window, width=6, textvariable=self._level)
        levelCombo['values'] = ("14", "15", "16", "17", "18", "19", "20")
        levelCombo.grid(column=0, row=2)
        levelCombo.current(2)

        labelLevel = ttk.Label(self._window, text="Level")
        labelLevel.grid(column=0, row=1)

        labelProgress = ttk.Label(self._window, text="Progress")
        labelProgress.grid(column=1, row=1)

        start_but = tk.Button(self._window, padx=10, pady=10, text="Start!", command=self.click_start)
        start_but.grid(column=2, row=2)

        self._window.mainloop()

    def click_start(self):
        '''If push 'Start' button, it check the file is .osm. and run cut func'''
        file_extension = self._file_path[-4:]
        
        if (file_extension == ".osm"):
            # super().__init__(int(self._level.get()), elemTree.parse(self._file_path).getroot()) # == super().__init__(DEGREE, doc_root)
            super(GUICutter, self).__init__(int(self._level.get()), elemTree.parse(self._file_path).getroot()) # == super().__init__(DEGREE, doc_root)

            self._directory_name = './Output/' + self._file_path.split('/')[-1] + '_' + self._level.get()   # self._file_path.split('/')[-1] == filename.osm
            self._create_folder(self._directory_name)

            spending_time = self.cut()

            # Inform spending_time & Initialize progressbar
            tkMessageBox.showinfo("Cutting Success!", 'Spending : ' + spending_time + 'sec')

            self._progressbar['value'] = 0
            self._window.update_idletasks()

        else:
            tkMessageBox.showinfo("Error", "Please Check Your Input File")
            
    def click_load(self):
        '''Load file_path'''
        self._file_path  = tkFileDialog.askopenfilename(filetypes=(("josm files","*.osm"), ("all files", "*.*")))
        self._file_dir_screen.insert(tk.END, self._file_path)

    def click_exit(self):
        '''Exit'''
        self._window.quit()
        self._window.destroy()
        exit()

    def cut(self):
        '''
        1. Scanning file by _scan_by_recursive func, then _tile_id_list filled
        2. Make piece by accord to _tile_id_list
        '''
        start = time.time()

        # 1. Scanning file by _scan_by_recursive func, then _tile_id_list filled
        nodes_list = self._doc_root.findall('./node')
        self._scan_by_recursive(nodes_list)

        # Calculate Progress (measure total working due)
        num_of_tile = len(self._tile_id_list)
        # # tot_progress = (num_of_tile*(num_of_tile+1) / 2)

        # 2. Make piece by accord to _tile_id_list
        for tile_id in self._tile_id_list:
            result = self._make_piece_by_id(tile_id)

            if result is not None:
                tree = elemTree.ElementTree(result)
                filename = "".join ([self._directory_name, '/', str(tile_id), '.osm'])
                tree.write(filename, encoding="utf-8", xml_declaration=True)

            # Show Progress
            # # self._progressbar.step(float(num_of_tile) * (tot_progress ** -1) * 100.0)
            # # num_of_tile -= 1
            self._progressbar.step((num_of_tile ** -1) * 100.0)
            self._window.update_idletasks()

        # return spending_time
        spending_time = str(round(time.time() - start, 2))

        return spending_time