FROM ros:humble-ros-base:65e1667


RUN apt-get update \
    && apt-get install -y libzmq3-dev libboost-dev qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev

RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git \
    && cd BehaviorTree.CPP \
    && git checkout tags/4.7.0 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j8 \
    && make install \
    && cd .. \
    && rm -rf BehaviorTree.CPP