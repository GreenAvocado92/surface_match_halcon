# surface_match_halcon

## 简介

此 `repo` 封装了 `halcon` 的 `c++` 版本的 `3D`模板匹配功能,仅供参考,若要使用,尚需关注商用软件的版权问题

## 配置

### 1. 安装`PCL`
在 `linux-ubuntu` 下安装命令为:
```
sudo apt install libpcl-dev
```

### 2. 安装`boost`
一般情况下,在安装完 `pcl` 后就已经安装完 `boost` 了,如果没有安装的话,可以采用以下命令安装:
```
sudo apt install libboost-dev
```

### 3. 安装 `halcon`
官网注册下载需要的版本,然后在`.bashrc`中配置下相关的环境变量即可,网上安装方式很多,不做赘述,以下是我的`.bashrc`部分内容,可做参考:

``` bash
HALCONARCH=x64-linux; export HALCONARCH
HALCONROOT=/opt/halcon; export HALCONROOT
HALCONEXAMPLES=${HALCONROOT}/examples; export HALCONEXAMPLES
HALCONIMAGES=${HALCONROOT}/examples/images; export HALCONIMAGES
PATH=${HALCONROOT}/bin/${HALCONARCH}:${HALCONROOT}/FLEXlm/${HALCONARCH}:${PATH}
export PATH
if [ ${LD_LIBRARY_PATH} ] ; then
LD_LIBRARY_PATH=${HALCONROOT}/lib/${HALCONARCH}:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH
else
LD_LIBRARY_PATH=${HALCONROOT}/lib/${HALCONARCH}; export LD_LIBRARY_PATH
fi

if [ "x${FLEXID_LIBRARY_PATH}" = "x" ]; then
FLEXID_LIBRARY_PATH="${HALCONROOT}/FLEXlm/${HALCONARCH}/flexid9:/usr/lib"
export FLEXID_LIBRARY_PATH
fi
```
相应的路径换成自己的本地路径,更新完记得
```
source ~/.bashrc
```

### 4. 编译运行

```
mkdir build 
cmake ..
make -j12
```

## 使用

**注意**:制作模板的点云文件需要手动处理的干净些,避免一些无效噪点的存在,且建议,所有的输入点云均采用`mm`为单位

### 1.参数介绍
```
("help", "show help information")
("mode", bpo::value<std::string>(), "mode, train or det")
("model", bpo::value<std::string>(), "input model file path")
("scene", bpo::value<std::string>(), "input scene file path")
("sfm", bpo::value<std::string>(), "set SFM file path")
("sd", bpo::value<float>(), "sample distance");
```


### 2.生成模板
```
./surface_match --mode train --model ../data/model_test_2.ply --sd 2.0
```


### 3. 场景识别
```
./surface_match --mode det --model ./model.ply --sd 0.01 --scene ./0-same-clear_mm.ply --sfm model.sfm
```

