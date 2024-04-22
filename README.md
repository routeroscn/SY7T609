# SY7T609
sy7t609 适用于 Arduino 的库
根据 https://github.com/baejinhoon-kor/IC
参考
https://github.com/shzlww/esphome_custom_components
 修改适用优化去除了一些问题
 
目前获取电量的好像有点问题没有研究了 

关于自动校正电压如果有稳定的电源参考值
比如当前已知电压 是 230v

程序可以第一次 启动的时候
// 因为是第一次自动校准0x038270 230v
sendCalibration(0x038270);

如果是手动 可以 sendCalibration(230000);
电流 也可以参考增加函数去校准 目前看电流不太需要校准
