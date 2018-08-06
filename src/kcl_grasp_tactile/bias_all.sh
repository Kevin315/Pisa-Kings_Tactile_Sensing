#!/bin/sh
for i in `rosservice list|grep bias`
do
rosservice call $i
done

