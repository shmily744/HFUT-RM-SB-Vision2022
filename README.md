# HFUT-RM-SB-Vision2021
合肥工业大学robomaster苍穹战队2021赛季烧饼视觉代码

主体识别部分采用了上交2021开源部分，自行添加了卡尔曼滤波预测。



被电控骂的很拉的通信：

```c++
string write_buffer = "A1" + num(to_string(p_p.x)) + " " + num(to_string(p_p.y)) + "B";
int bytes_written = write(fd, write_buffer.c_str(), write_buffer.size()); //传出坐标
```

A、B为帧头帧尾

考虑到设置分辨率x，y坐标均设为三位

A后1为识别到目标，0为未识别到，后接**三位数x坐标 +空格+三位数y坐标**再接帧尾

例`识别到：A1045 155B`   `未识别：A0000 000B`

