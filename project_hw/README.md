## 开发环境要求：
Ubuntu  ≥ 	18.04
cmake	≥  	3.27.5
make	≥ 	4.1
GCC		≥ 	7.5

## 编译运行

```shell
# 命令
cd project_hw
mkdir build
cd build
cmake ..
make
./project_hw -f /home/project_hw/dataset/case_1.txt
```



```shell
# 示例
[root@kwepwebenv20531 project_hw]# mkdir build
[root@kwepwebenv20531 project_hw]# cd build/
[root@kwepwebenv20531 build]# cmake ..
[root@kwepwebenv20531 build]# make
Scanning dependencies of target project_hw
[ 33%] Building C object CMakeFiles/project_hw.dir/main.c.o
[ 66%] Building C object CMakeFiles/project_hw.dir/algorithm/algorithm.c.o
[100%] Linking C executable project_hw
[100%] Built target project_hw
[root@kwepwebenv20531 build]# ll
总用量 48
-rw-r--r--. 1 root root 14459 7月  11 15:13 CMakeCache.txt
drwxr-xr-x. 5 root root   235 7月  11 15:13 CMakeFiles
-rw-r--r--. 1 root root  1633 7月  11 15:13 cmake_install.cmake
-rw-r--r--. 1 root root  6094 7月  11 15:13 Makefile
-rwxr-xr-x. 1 root root 17360 7月  11 15:13 project_hw
[root@kwepwebenv20531 build]# 
[root@kwepwebenv20531 build]# ./project_hw -f /home/project_hw/dataset/case_4.txt
Welcome to HW project.
The file path is: /home/project_hw/dataset/case_4.txt
head info : [5,1000,0]

io count = 10
input io array:
io [1] : [1,4,100,150]
io [2] : [2,125,58,5]
io [3] : [3,9,90,30]
io [4] : [4,3,29,120]
io [5] : [5,6,500,1500]
io [6] : [6,4,100,150]
io [7] : [7,125,58,5]
io [8] : [8,9,90,30]
io [9] : [9,3,29,120]
io [10] : [10,6,500,1500]


Key Metrics:
	algorithmRunningDuration:	 45.000000 ms
	addressingDuration:		 123 ms
	readDuration:			 456 ms
	tapeBeltWear:			 200
	tapeMotorWear:			 100
output sequence: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, ]
[root@kwepwebenv20531 build]# 
```

