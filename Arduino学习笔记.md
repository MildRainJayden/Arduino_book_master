Arduino学习笔记

## 1. Blink (Arduino的hello world)

``` c
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```

## 2. 基础图表速查

### 1. 整型与取值范围

![](E:\桌面\Markdown笔记空间\引用文件\IMG_0244.PNG)

### 2.静态量

在Arduino核心库中，OUTPUT被定义为1，INPUT被定义为0，HIGH被定义为1，LOW被定义为0

### 3.ADC参考电压可用配置
![](E:\桌面\Markdown笔记空间\引用文件\adc参考电压可用配置.JPG)

### 4.中断引脚与中断编号
![](E:\桌面\Markdown笔记空间\引用文件\中断引脚与中断编号.JPG)

### 5.中断模式
![](E:\桌面\Markdown笔记空间\引用文件\可用的中断触发模式.JPG)

### 6.IIC总线(TWI)
![](E:\桌面\Markdown笔记空间\引用文件\IIC.JPG)

### 7.SPI引脚
![](E:\桌面\Markdown笔记空间\引用文件\SPI.JPG)

### 8.数字电位器AD5206引脚配置

![](E:\桌面\Markdown笔记空间\引用文件\IMG_0303.JPG)

![IMG_0304](E:\桌面\Markdown笔记空间\引用文件\IMG_0304.JPG)

![IMG_0305](E:\桌面\Markdown笔记空间\引用文件\IMG_0305.JPG)

### 9.74HC595扩展I/0口

![](E:\桌面\Markdown笔记空间\引用文件\IMG_0306.JPG)

<img src="E:\桌面\Markdown笔记空间\引用文件\IMG_0307.JPG">

![IMG_0308](E:\桌面\Markdown笔记空间\引用文件\IMG_0308.JPG)

### 10.SD卡引脚设置

![](E:\桌面\Markdown笔记空间\引用文件\IMG_0310.JPG)

### 11.LCD1602引脚设置

![LCD1602](E:\桌面\Markdown笔记空间\引用文件\LCD1602.png)

### 12.DS1307引脚设置

![IMG_0316](E:\桌面\Markdown笔记空间\引用文件\IMG_0316.JPG)

### 13.MINI 12864引脚设置

![IMG_0318](E:\桌面\Markdown笔记空间\引用文件\IMG_0318.JPG)

![IMG_0319](E:\桌面\Markdown笔记空间\引用文件\IMG_0319.JPG)





## 3. 函数语法分析

``` c 
pinMode(pim,mode);//配置引脚的模式为输入模式或输出模式
digitalWrite(pin,value);//使引脚pin输出高电平或低电平
digitalRead(buttonPin);//读取引脚pin的电平状态
Serial.begin(speed);//串口波特率设置，Serial类为串口类
Serial.println(val);//串口输出数据，ln为多输出一组回车换行符
char ch=Serial.read();//接收串口数据,串口缓冲区无可读数据时返回-1（乱码）
Serial.available();//搭配while/if语句判断缓冲区中是否有数据>0
analogRead(pin);//ADC模拟输入
analogWrite(pin,value);//ADC模拟输出，pwm，value值为脉冲宽度，0~255，引脚带~
//获取Arduino从通电或复位后到现在的时间
millis();//返回unsigned long 单位毫秒 50天溢出一次
micros();//返回unsigned long 单位微秒 精度为晶振MHz数

//延时函数
delay();//返回unsigned long 毫秒级延时
delayMicroseconds();//返回unsigned int 微秒级延时

//调声函数
tone(pin,frequency,duration);//产生占空比50%的指定频率方波，duration单位ms
noTone();//停止
pulseIn(pin,valuie,timeout);//测定指定引脚上的脉冲信号宽度，返回unsigned long微秒，timeout超时时间，未设置默认1秒
analogReference(type);//设置使用外部参考电压

//中断函数
attachInterrupt(interrupt,function,mode);//interrupt中断编号，function中断函数名，mode中断模式
attachInterrupt(interrupt);//禁用外部中断，interrupt，需要禁用的中断编号

//HardwareSerial类库成员函数，使用时加前缀Serial.xxx();
available();//获取串口接收到的数据个数看，缓冲区最多保存64B
begin(speed,config可略);//初始化串口，配置参数，范例，Serial.begin(9600,SERIAL 8E2)语句设置串口波特率为9600，数据位为9，偶校验，停止位2
end();//结束串口通信，释放引脚
find(target);//从串口缓冲区读取数据，直到读到指定的字符串,boolean值
findUnit(target,terminal);//从串口缓冲区读取数据直到读到指定的字符串或指定的停止符
flush();//等待正在发送的数据发送完成
parseFloat();//从串口缓冲区返回第一个有效的float数据
parseInt();//从串口缓冲区返回第一个有效的int数据
peek();//返回1字节的数据，但不会从接收缓冲区删除该数据
print(val,format);//将数据输出到串口,ASCII码
println(val,format);//将数据输出到串口，并回车换行，ASCII码
read();//返回1字节的数据，但会从接收缓冲区删除该数据
readBytes(buffer,length);//从接收缓冲区读取指定长度的字符，并将其存入一个数组中，有返回值，没找到有效数据，返回0
readBytesUntil(character,buffer,length);//从接收缓冲区读取指定长度的字符，并将其存入一个数组中，若遇到停止符或超时则退出函数，返回0
setTimeout();//设置上述两个函数的超时时间，单位毫秒
write();//输出数据到串口，字节形式，语法(val),(str),(buf,len)

//processing串口通信库
void serialEvent(){
    //当串口接收缓冲区中有数据时，会触发该事件
    //仅仅是一个运行在两次loop()之间的函数
}

//SoftwareSerial类库 软件模拟串口通信 需要先声明SoftwareSerial.h
SoftwareSerial(rxPin,txPin);//SoftwareSerial类的构造函数，指定rx、tx引脚
//SoftwareSerial mySerial=SoftwareSerial(rxPin,txPin);
mySerial.listen();//开启监听
mySerial.isListening();//检测软串口是否处于监听状态 boolean
mySerial.overflow();//检测缓冲区是否已经溢出 boolean

//Wire类库
begin(address);//初始化IIC链接，作为主机或从机 主机无参数，从机adress0~127
Wire.requestForm(addressk,quantity,stop);//主机向从机发送数据请求信号,使用requestForm()后，从机端可以使用onRequest()注册一个事件用以响应主机的请求，主机可以通过available()和read()函数读取这些数据，quantity请求的字节数，stop,boolean，true时发送一个停止信息，释放IIC总线，当为false时，将发送一个重新开始信息，并继续保持IIC总线的有效连接
Wire.beginTransmission(address);//设定传输数据到指定的从机设备，随后可以用write()函数发送数据，并搭配endTransmission()函数结束数据传输
Wire.endTransmission(stop);//结束数据传输，stop,boolean,默认true，返回值，byte值，表示本次传输状态：0成功，1数据过长，2在地址发送时接收到NACK信号，3在数据发送时接收到NACK信号，4其他错误
Wire.write(value);Wire.write(string);Wire.write(data,length);//value以单字节发送，string以一系列字节发送，data以字节形式发送数组，length传输的字节数，返回byte值，返回输入的字节数
Wire.available();//返回接收到的字节数，在主机中，一般用于主机发送数据请求后；在从机，一般用于数据接收事件中
Wire.read();//读取1B的数据 
Wire.onReceive(handler);//该函数可在从机端注册一个事件，当从机收到主机发送的数据即被触发，handler，当从机接收到数据时可被触发的事件，该事件带有一个int型参数（从主机读到的字节数）且没有返回值，如void myHandler(int numBytes)
Wire.onRequest(handler);//注册一个事件，当从机接收到主机的数据请求时即被触发，handler，可被触发的事件，该事件不带参数和返回值，如void muHandler()

//SPI类库成员函数
SPI.begin();//初始化SPI通信，调用该函数后，SCK,MOSI,SS引脚将被设置为输出模式，且SCK和MOSI引脚被拉低，SS引脚被拉高
SPI..end();//关闭SPI总线通信
SPI.setBitOrder(order);//设置传输顺序,order，取值LSBFIRST，低位在前，MSBFIRST,高位在前
SPI.setClockDivider(divider);//设置通信时钟，时钟信号由主机产生，从机不用配置，但主机的SPI时钟频率应该在从机允许的处理速度范围内,divider,SPI通信的时钟是由系统时钟分频得到的，可使用的分频配置为：SPI_CLOCK_DIV2/4(默认)/8/16/32/64/128
SPI.setDataMode(mode);//设置数据模式，mode包括SPI_MODE0/1/2/3
SPI.transfer(val);//传输1B数据，参数为发送到数据
SPISettings.mySetting(speed,dataOrder,datamode);//使用新的SPI库，每个SPI设备可以被配置一次作为一个SPISettings对象，speed通讯的速度，dataOrder取MSBFIRST或LSBFIRST，datamode同mode
SPI.beginTransaction(mySettings);//初始化使用SPISettings定义的SPI总线
SPI.endTransaction();//停止使用SPI总线，允许SPI总线使用其他库
SPI.usinginterrupt(InterruptNumber);//在中断中使用SPI，参数代表中断号
//软件模拟SPI通信
shiftOut(dataPin,clockPin,bitOrder,value);//模拟SPI串行输出 dataPin数据输出引脚，clockPin时钟输出引脚，bitOrder数据传输顺序，value传输的数据
shiftIn(dataPin,clockPin,bitOrder);//模拟SPI串行输入 dataPin数据输入引脚，clockPin时钟输入引脚，bitOrder数据传输顺序，返回值：输入的串行数据

//EEPROM类库成员函数 EEPROM.h
EEPROM.write(address,value);//对指定地址写入数据
EEPROM.read(address);//从指定地址读出数据，一次读/写1B数据，如果指定的地址没有写入过数据，则读出值为225，返回byte型
EEPROM.update(address,value);//写一个字节到EEPROM，该值如果不同于已保存在同一地址的值时才会写入(更新该地址的值)
EEPROM.get(address,data);//从EEPROM中读取任何类型的数据或对象，address读取的位置，从0开始int整型，data，读取的数据，可以是一个基本类型，或者是一个自定义的结构体，返回值：数据传递的引用
EEPROM.put(address,data);//向EEPROM中写入任何类型数据或对象，address写入的位置，从0开始int整型，data写入的数据，可以是一个基本类型，或者是一个自定义的结构体，返回值：数据传递的引用，注意：这个函数使用EEPROM.update()来执行写数据，所以如果值没有改变不会重复写入
EEPROM[address]/*这个操作符允许像一个数组一样使用标识符'EEPROM'。使用这种方式可以直接对EEPROM单元进行读取和写入操作，address读/写位置，从0开始int型，返回值：A reference to the EEPROM cell*/

//SD卡类库的使用 需要包含SPI.h和SD.h SD卡库包含SDClass类和File类
//SDClass类
SD.begin(cspin);//初始化SD卡库和SD卡，当使用SD.begin()时，默认将Arduino SPI的 SS引脚连接到SD卡的CS使能选择端；也可以使用begin(cspin)指定一个引脚连接到SD卡的CS 使能选择端，但仍需保证SPI的SS引脚为输出模式，否则SD卡库将无法运行。参数：cspin,连接到SD卡CS端的Arduino引脚。返回值：boolean型值，为true表示初始化成功；为false表示初始化失败。
SD.exists(filename);//检查文件或文件夹是否存在于SD卡中
SD.open(filename,mode);//打开SD卡上的一个文件，如果文件不存在，且以写入方式打开，则会创建一个指定文件名的文件，mode(可选)：FILE_READ只读方式,FILE_WRITE写入方式
SD.remove(filename);//从SD卡移除一个文件，如果文件不存在，则函数返回值是不确定的，因此在移除文件之前，最好使用SD.exists(filename)先检测文件是否存在
SD.mrdir(filename);//创建文件夹
SD.rmdir(filename);//移除文件夹，被移除的文件夹必须是空的
//File类  file，一个File类型的对象
file.available();//检查当前文件中可读数据的字节数
file.close();//关闭文件，并确保数据已经被完全写入SD卡中
file.flush();//确保数据已经写入SD卡，当文件被关闭时，flush()会自动运行
file.peek();//读取当前所在字节，但并不移动到下一字节
file.position();//获取当前在文件夹中的位置(即下一个被读/写的字节的位置)
file.print(data,BASE);//输出数据到文件，要写入的文件应该已经被打开，且等待写入 BASE(可选)指定数据输出形式
file.println(data,BASE);//输出数据到文件，并回车换行
file.seek(pos);//跳转到指定位置，该位置必须在0到文件大小之间 pos需要查找的位置 返回boolean值
file.size();//获取文件的大小
file.read();//读取1B数据
file.write(data);file.write(buf,len);//写入数据到文件，buf一个字符数组或字节数据，len，buf数组的元素个数
file.isDirectory();//判断当前文件是否为目录
file.openNextFile();//打开下一个文件
file.rewindDirectory();//回到当前目录中的第一个文件

//IRremote类库
//IRrecv类
IRrecvobject(recvpin);//IRrecv类的构造函数，可用于指定红外一体化接收头的连接引脚，object用户自定义的对象名，recvpin连接到红外一体化接收头的引脚编号
IRrecv.enableIRIn();//初始化红外解码
IRrecv.decode(&results);//IRrecv一个IRrecv类的对象，results一个decode_results类的对象 返回值1或0
IRrecv.resume();//接收下一个编码，IRrecv一个IRrecv类的对象
//IRsend类 IRsend，一个IRsend类的对象
IRsendobject();//IRsend类的构造函数 object一个IRsend类的对象
IRsend.sendNEC(data,nbits);//以NEC编码格式发送指定值 data发送的编码值，nbits编码位数
IRsend.sendSony(data,nbits);//以Sony编码格式发送指定值
IRsend.sendRaw(buf,len,hz);//发送原始红外编码信号 IRsend一个IRsend类的对象，buf存储原始编码的数组，len数组长度，hz红外发射频率 其他函数的常见协议如RC5,RC6,DISH,Sharp,Panasonic,JVC(替换Raw)

//LiquidCrystal类库 LiquidCrystal.h
(接下图)
```

![IMG_0312](C:\Users\11541\Desktop\Markdown笔记空间\引用文件\IMG_0312.JPG)

![IMG_0313](C:\Users\11541\Desktop\Markdown笔记空间\引用文件\IMG_0313.JPG)

![IMG_0314](C:\Users\11541\Desktop\Markdown笔记空间\引用文件\IMG_0314.JPG)

![A1F5100464F88A7CA83C1D0B5ED116E4](C:\Users\11541\Desktop\Markdown笔记空间\引用文件\A1F5100464F88A7CA83C1D0B5ED116E4.png)

```c
//u8glib类库
//成员函数较多，见下列链接
```

[u8glib]([u8glib | XX到此一游 (clz.me)](https://clz.me/u8glib/))

```c
//USB类库与Ethernet类库由于硬件限制暂不学习 需要leonardo和ethernet扩展版
```

## 4.备注

```c
Serial.read(); //读取串口类型是整型，但默认是ASCII码，想输出原整数：
//只需使用.toInt（）函数。
//您应该从串口读取字符串，然后将其转换为整数。
Serial.print(Serial.readString().toInt()); 
```





# 项目志

## LED流水灯

备注： 使用LED灯规格： 白蓝：2.2v-2.4v

​                                          红绿黄：1.8v-2.0v
​                                          一般LED最大承受电流为25mA

短接引脚输出高电平时电流值为84mA

物料清单：UNO、面包板、6xLED、6x220Ω电阻

``` c

void setup() {
for(int i=2;i<=7;i++){
  pinMode(i, OUTPUT);
}
int i=2;
}

void loop() {
  
// 流水灯 左右返回重复
  for(int i=2;i<=7;i++){
  digitalWrite(i, HIGH);  
  delay(1000);                      
  digitalWrite(i, LOW);   
  }    
  for(int i=7;i>=2;i--){
  digitalWrite(i, HIGH);  
  delay(1000);                      
  digitalWrite(i, LOW);   
  }          
//流水灯 从两边到中间
int j=0;
int sum=6;//LED灯总数
for(int i=1;i<=sum/2;i++)
{
  j=sum+1-i;
  digitalWrite(i+1, HIGH);  
  digitalWrite(j+1, HIGH); 
  delay(1000);                      
  digitalWrite(i+1, LOW); 
  digitalWrite(j+1, LOW); 
}
//流水灯 从中间到两边
for(int i=sum/2;i>=1;i--)
{
  j=sum+1-i;
  digitalWrite(i+1, HIGH);  
  digitalWrite(j+1, HIGH); 
  delay(1000);                      
  digitalWrite(i+1, LOW); 
  digitalWrite(j+1, LOW); 
}

}
```







## 按键控制led灯

下拉电阻与限流电阻，其中下拉电阻的作用是稳定悬空引脚的电平
于此类似的，内部上拉电阻

``` c

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);//接外部下拉电阻时
  pinMode(buttonPin, INPUT_PULLUP);//使能引脚上的内部上拉电阻  
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

   /*一个很好的程序思想，等待按键按下，使用while循环，状态检测
   while(digitalRead(buttonPin)==HIGH){}
  */
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
    delay(500);//末尾延时，程序运行极快，防止按键冲突
}
```

```c
//按键控制LED 每按一次切换亮灭状态

int buttonPin = 2;
int ledPin = 7;
boolean ledState = false;   // 记录LED状态
boolean buttonState = true; // 记录按键状态

void setup()
{
  //初始化I/O口
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  //等待按键按下
  while (digitalRead(buttonPin) == HIGH)
  {
  }

  //当按键按下时，点亮或熄灭LED
  if (ledState == true)
  {
    digitalWrite(ledPin, LOW);
    ledState = !ledState;
  }
  else
  {
    digitalWrite(ledPin, HIGH);
    ledState = !ledState;
  }
  delay(500);
}
```



## 人体感应灯

主要物料清单：继电器、人体热释电红外传感器、灯

``` c
int PIRpin=2;
int RELAYpin=3;

void setup(){
    Serial.begin(9600);
    pinMode(PIRpin,INPUT);
    pinMode(RELAYpin,OUTPUT);
}

void loop(){
    //等待传感器检测到人,会传输一个高脉冲
    while(!digitalRead(PIRpin)){}
    //将灯打开10秒，然后关闭
    Serial.println("turn on");
    digitalWrite(RELAYpin,HIGH);
    delay(10000);
    digitalWrite(RELAYpin,LOW);
    Serial.println("turn off");
} 
```

## 呼吸灯

物料清单：LED、220Ω电阻

``` c
int ledPin = 9;    // LED connected to digital pin 9

void setup() {
  // nothing happens in setup
}

void loop() {
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}
```

另一种使用电位器调节呼吸灯频率略

通过光敏电阻实现检测环境光亮度实验

## 电子温度计

通过lm35温度传感器输出的模拟值，换算为对应的摄氏温度公式：

lm35 0~100°C 每升温1°C 电压升高10mV

``` c
float temp=(5.0*analogRead(LM35)*100.0)/1024;
```

```c
#include "DHT.h"
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
}
```



## 串口控制开关灯

备注：uno自带一个名为L的13号引脚控制的led灯

``` c
void setup(){
    Serial.begin(9600);
    pinMode(13,OUTPUT);
}
void loop(){
    if(Serial.available()>0){
        char ch=Serial.read();
        Serial.print(ch);
        //开灯
        if(ch=='a')
        {
            digitalWrite(13,HIGH);
            Serial.println("turn on");
        }
        else if(ch=='b')
        {
            digitalWrite(13,LOW);
            Serial.println("turn off");
        }
    }
}
```

## 电子琴

``` c
#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}

void loop() {
  // no need to repeat the melody.
}
```

## 超声波测距

物料清单：SR04

测距公式：340m/s*t/2

``` c
const int TrigPin=2;
const int EchoPin=3;
float distance;

void setup(){
    Serial.begin(9600);
    pinMode(TrigPin,OUTPUT);
    pinMode(EchoPin,INPUT);
    Serial.println("Ultrasonic sensor:");
}

void loop(){
    //产生一个10us的高脉冲去触发TrigPin
    digitalWrite(Trigpin,LOW);
    delayMicroseconds(2);
    digitalWrite(Trigpin,HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigpin,LOW);
    //检测脉冲宽度，并计算出距离
    distance=pulseIn(EchoPin,HIGH)/58.00;
    Serial.print(distance);
    Serial.print("cm");
    Serial.println();
    delay(1000);
}
```

## 外部中断触发蜂鸣器报警

物料清单：数字红外障碍传感器(发出调制后的38kHz红外光)，有障碍物时输出低电平，无障碍物时输出高电平

``` c
//默认无遮挡时蜂鸣器发声
boolean RunBuzzer=true;

void setup(){
    Serial.begin(9600);
    //初始化外部中断
    //当int0的电平有高变低时，触发中断函数warning()
    attachInterruption(0,warning,CHANGE);
}

void loop(){
    if(RunBuzzer)
    {
        tone(8,1000);
    }
    else
    {
        noTone(8);
    }
}

//中断函数
void warning()
{
    RunBuzzer=!RunBuzzer;
}
```

## 编写Arduino类库

备注：编写SR04为例

``` c
//编写SR04.h头文件
#ifndef SR04_H
#define SR04_H
#if defined(ARDUINO)&&ARDUINO>=100
   #include "Arduino.h"
#else
   #include "WProgram.h"
#endif

class SR04{
    public:
        SR04(int TrigPin,int EchoPin);
        float Get();
    private:
        int Trig_Pin;
        int Echo_Pin;
        float distance;
};
#endif
```

``` c
//以下条件编译命令目的是增加Arduino IDE的版本兼容性
#if defined(ARDUINO)&&ARDUINO>=100
   #include "Arduino.h"
#else
   #include "WProgram.h"
#endif
//若版本号在1.0后,则调用WProgram.h(主要函数声明头文件)
```

## print()和write()输出方式的差异

``` c
float FLOAT = 1.23456;
int INT = 123;
byte BYTE[6] = {48, 49, 50, 51, 52, 53};

void setup()
{
    Serial.begin(9600);
    //Print的各种输出形式
    Serial.println("Serial Print:");
    Serial.println(INT);
    Serial.println(INT, BIN);
    Serial.println(INT, OCT);
    Serial.println(INT, DEC);
    Serial.println(INT, HEX);
    Serial.println(FLOAT);
    Serial.println(FLOAT, 0);
    Serial.println(FLOAT, 2);
    Serial.println(FLOAT, 4);

    //Write的各种输出形式
    Serial.println();
    Serial.println("Serial Write:");
    Serial.write(INT);
    Serial.println();
    Serial.write("Serial");
    Serial.println();
    Serial.write(BYTE, 6);
}

void loop()
{
}
```

## read()和peek()输入方式的差异

```c
//read函数读取串口数据
char col;
void setup()
{
    Serial.begin(9600);
}

void loop()
{
    while (Serial.available() > 0)
    {
        col = Serial.read();
        Serial.print("Read: ");
        Serial.println(col);
        delay(1000);
    }
}

//peek函数读取串口数据

char col;
void setup()
{
    Serial.begin(9600);
}

void loop()
{
    while (Serial.available() > 0)
    {
        col = Serial.peek();
        Serial.print("Read: ");
        Serial.println(col);
        delay(1000);
    }
}
```

## 串口读取字符串

```c
void setup()
{
    Serial.begin(9600);
}

void loop()
{
    String inString = "";
    while (Serial.available() > 0)
    {
        char inChar = Serial.read();
        inString += (char)inChar;
        //延时函数用于等待输入字符完全进去接收缓冲区
        delay(10);
    }
    // 检查是否接收到数据，如果接收到，便输出该数据
    if (inString != "")
    {
        Serial.print("Input String: ");
        Serial.println(inString);
    }
}
```

## 串口事件

``` c
String inputString = "";        // 用于保存输入数据的字符串
boolean stringComplete = false; // 字符串是否已接收完全

void setup()
{
    // 初始化串口
    Serial.begin(9600);
    // 设置字符串存储量为200个字节
    inputString.reserve(200);
}

void loop()
{
    // 当收到新的一行字符串时，输出这个字符串
    if (stringComplete)
    {
        Serial.println(inputString);
        // 清空字符串
        inputString = "";
        stringComplete = false;
    }
}

/*
当一个新的数据被串口接收到时会触发SerialEvent事件
  SerialEvent函数中的程序会在两次loop()之间运行
因此，如果你loop中有延时程序，会延迟该事件的响应。
造成数个字节的数据都可以被接收。
*/
void serialEvent()
{
    while (Serial.available())
    {
        // 读取新的字节
        char inChar = (char)Serial.read();
        // 将新读取到的字节添加到inputString字符串中
        inputString += inChar;
        // 如果收到换行符，则设置一个标记
        // 再在loop()中检查这个标记，用以执行相关操作
        if (inChar == '\n')
        {
            stringComplete = true;
        }
    }
}
```

## 串口控制RGB调光

```c 
int i;                          //保存PWM需要输出的值
String inString = "";           // 输入的字符串
char LED = ' ';                 // 用于判断指定LED颜色对应的引脚
boolean stringComplete = false; // 用于判断数据是否读取完成

void setup()
{
    //初始化串口
    Serial.begin(9600);
}
void loop()
{
    if (stringComplete)
    {
        if (LED == 'A')
        {
            analogWrite(9, i);
        }
        else if (LED == 'B')
        {
            analogWrite(10, i);
        }
        else if (LED == 'C')
        {
            analogWrite(11, i);
        }
        // 清空数据，为下一次读取做准备
        stringComplete = false;
        inString = "";
        LED = ' ';
    }
}

//使用串口事件
// 读取并分离字母和数字
void serialEvent()
{
    while (Serial.available())
    {
        // 读取新的字符
        char inChar = Serial.read();
        //根据输入数据分类
        // 如果是数字，则存储到变量inString中
        // 如果是英文字符，则存储到变量LED中
        // 如果是结束符“\n”，则结束读取，并将inString转换为int
        if (isDigit(inChar))
        {
            inString += inChar;
        }
        else if (inChar == '\n')
        {
            stringComplete = true;
            i = inString.toInt();
        }
        else
            LED = inChar;
    }
}
```

## Arduino间的串口通信

```c 
//A
/*
Arduino Mega端程序
串口使用情况：
Serial ------ computer
Serial1 ------ UNO SoftwareSerial
*/

void setup()
{
    // 初始化Serial，该串口用于与计算机连接通信
    Serial.begin(9600);
    // 初始化Serial1，该串口用于与设备B进行连接通信
    Serial1.begin(9600);
}

// 两个字符串分别用于存储AB两端传输来的数据
String device_A_String = "";
String device_B_String = "";

void loop()
{
    // 读取从计算机传入的数据，并通过Serial1发送给设备B
    if (Serial.available() > 0)
    {
        if (Serial.peek() != '\n')
        {
            device_A_String += (char)Serial.read();
        }
        else
        {
            Serial.read();
            Serial.print("you said: ");
            Serial.println(device_A_String);
            Serial1.println(device_A_String);
            device_A_String = "";
        }
    }

    //读取从设备B传入的数据，并在串口监视器中显示
    if (Serial1.available() > 0)
    {
        if (Serial1.peek() != '\n')
        {
            device_B_String += (char)Serial1.read();
        }
        else
        {
            Serial1.read();
            Serial.print("device B said: ");
            Serial.println(device_B_String);
            device_B_String = "";
        }
    }
}
//B
/*
Arduino UNO端程序
串口使用情况：
Serial ------ computer
softSerial ------ Mega Serial1
*/

#include <SoftwareSerial.h>
//新建一个softSerial对象，RX:10，TX:11
SoftwareSerial softSerial(10, 11);

void setup()
{
    //初始化串口通信
    Serial.begin(9600);
    //初始化软串口通信
    softSerial.begin(9600);
    // 监听软串口通信
    softSerial.listen();
}

// 两个字符串分别用于存储AB两端传输来的数据
String device_B_String = "";
String device_A_String = "";

void loop()
{
    // 读取从计算机传入的数据，并通过softSerial发送给设备B
    if (Serial.available() > 0)
    {
        if (Serial.peek() != '\n')
        {
            device_B_String += (char)Serial.read();
        }
        else
        {
            Serial.read();
            Serial.print("you said: ");
            Serial.println(device_B_String);
            softSerial.println(device_B_String);
            device_B_String = "";
        }
    }
    //读取从设备A传入的数据，并在串口监视器中显示
    if (softSerial.available() > 0)
    {
        if (softSerial.peek() != '\n')
        {
            device_A_String += (char)softSerial.read();
        }
        else
        {
            softSerial.read();
            Serial.print("device A said: ");
            Serial.println(device_A_String);
            device_A_String = "";
        }
    }
}
```

## IIC主机写数据，从机接收数据

```c 
// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>
// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this
// Created 29 March 2006
// This example code is in the public domain.

#include <Wire.h>

void setup()
{
//作为主机加入到IIC总线
  Wire.begin(); 
}

byte x = 0;

void loop()
{
  Wire.beginTransmission(4); //向地址为4的从机传送数据
  Wire.write("x is ");        // 发送5个字节的字符串
  Wire.write(x);              // 发送1个字节的数据
  Wire.endTransmission();    // 结束传送
  x++;
  delay(500);
}
// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>
// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this
// Created 29 March 2006
// This example code is in the public domain.

#include <Wire.h>

void setup()
{
//作为从机加入IIC总线，从机地址为4
  Wire.begin(4); 
  //注册一个IIC事件
  Wire.onReceive(receiveEvent);
  //初始化串口
  Serial.begin(9600); 
}

void loop()
{
  delay(100);
}

// 当主机发送的数据被接收到时，将触发receiveEvent事件
void receiveEvent(int howMany)
{
// 循环读取接收到的数据，最后一个数据单独读取
  while(1 < Wire.available())
  {
    char c = Wire.read(); // 以字符形式接收数据
    Serial.print(c);       // 串口输出该字符
  }
  int x = Wire.read();    // 以整型形式接收数据
  Serial.println(x);      //串口输出该整形变量
}
```

## IIC从机发送数据，主机读取数据

```c 
// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>
// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this
// Created 29 March 2006
// This example code is in the public domain.

#include <Wire.h>

void setup()
{
//作为主机加入IIC总线
  Wire.begin();        
  Serial.begin(9600); // 初始化串口通信
}

void loop()
{
  Wire.requestFrom(2, 6); // 向2号从机请求6个字节的数据
  while(Wire.available())// 等待从机发送完数据
  { 
    char c = Wire.read(); // 将数据作为字符接收
    Serial.print(c);       // 串口输出字符
  }
  delay(500);
}

// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>
// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this
// Created 29 March 2006
// This example code is in the public domain.

#include <Wire.h>

void setup()
{
// 作为从机加入IIC总线，并将地址设为2
Wire.begin(2);  
// 注册一个事件，用于相应主机的数据请求
Wire.onRequest(requestEvent); 
}

void loop()
{
  delay(100);
}

// 每当主机请求数据时，该函数便会执行
// 在setup()中，该函数被注册为一个事件
void requestEvent()
{
  Wire.write("hello "); // 用6个字节的信息回应主机的请求，hello后带一个空格
}
```

## 使用数字电位器AD5206

物料清单：AD5206(6通道、256位程序控制电位器)

```C 
/*
  Digital Pot Control

  This example controls an Analog Devices AD5206 digital potentiometer.
  The AD5206 has 6 potentiometer channels. Each channel's pins are labeled
*/

// 引用SPI库
#include <SPI.h>

// 设置10号引脚控制AD5206的SS脚
const int slaveSelectPin = 10;

void setup() {
  // 设置SS为输出
  pinMode (slaveSelectPin, OUTPUT);
  // 初始化SPI
  SPI.begin(); 
}

void loop() {
  // 分别操作6个通道的数字电位器
  for (int channel = 0; channel < 6; channel++) { 
    // 逐渐增大每个通道的阻值
    for (int level = 0; level < 255; level++) {
      digitalPotWrite(channel, level);
      delay(10);
    }
    // 延时一段时间
    delay(100);
    // 逐渐减小每个通道的阻值
    for (int level = 0; level < 255; level++) {
      digitalPotWrite(channel, 255 - level);
      delay(10);
    }
  }
}

int digitalPotWrite(int address, int value) {
  // 将SS输出低电平，选择能这个设备
  digitalWrite(slaveSelectPin,LOW);
  // 向SPI传输地址和对应的配置值
  SPI.transfer(address);
  SPI.transfer(value);
  //将SS输出高电平，取消选择这个设备
  digitalWrite(slaveSelectPin,HIGH); 
}
```

## 使用74HC595扩展I/0口

物料清单：74HC595（输出端口扩展）

``` c
/*
使用74HC595扩展I/O
（shiftOut串行输出的使用）
*/
// ST_CP连接8号引脚
int latchPin = 8;
// SH_CP 连接12号引脚
int clockPin = 12;
// DS连接11号引脚
int dataPin = 11;

void setup() {
  //初始化模拟SPI引脚
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

void loop() 
{
  //count up routine
  for (int j = 0; j < 256; j++) 
  {
    //当你传输数据时，STCP需要保持低电平
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, j);   
// 传输完数据后，将STCP拉高
// 此时74HC595会更新并行引脚输出状态
    digitalWrite(latchPin, HIGH);
    delay(1000);
  }
}
```

## EEPROM写入操作

``` c
/*
* EEPROM Write
* Stores values read from analog input 0 into the EEPROM.
* These values will stay in the EEPROM when the board is
* turned off and may be retrieved later by another sketch.
*/

#include <EEPROM.h>

// EEPROM 的当前地址,即你将要写入的地址,这里就是从0开始写
int addr = 0;

void setup()
{
}

void loop()
{
  //模拟值读出后是一个0-1024的值，但每字节的大小为0-255
//所以这里将值除以4再存储到val
  int val = analogRead(0) / 4;

  // 写入数据到对应的EEPROM空间
  // 即使Arduino断电，数据也会保存在EEPROM里
  EEPROM.write(addr, val);

  // 逐字节写入数据
  // 当EEPROM空间写满后，重新从0地址开始写
  addr = addr + 1;
  if (addr == 512)
    addr = 0;
  delay(100);
}
```

## EEPROM读取操作

``` c
/*
* EEPROM Read
* Reads the value of each byte of the EEPROM and prints it 
* to the computer.
* This example code is in the public domain.
*/

#include <EEPROM.h>

// 从地址0处开始读取EEPROM的值
int address = 0;
byte value;

void setup()
{
  //初始化串口，并等待计算机打开串口
  Serial.begin(9600);
  while (!Serial) {
    ; //等待串口连接，该功能仅支持Arduino Leonardo
  }
}

void loop()
{
  // 从当前EEPROM地址中读取一个字节数据
  value = EEPROM.read(address);

  Serial.print(address);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();

  // 前进到下一个EEPROM地址
  address = address + 1;

  //当读到EEPROM尾部时，跳转到起始地址0处
  if (address == 512)
    address = 0;

  delay(500);
}
```

## EEPROM清除操作

``` c
/* * EEPROM Clear
* Sets all of the bytes of the EEPROM to 0.
* This example code is in the public domain.
*/

#include <EEPROM.h>

void setup()
{
  // 让EEPROM的512字节内容全部清零
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);

  // 清零工作完成后，将L灯点亮，提示EEPROM清零完成
  digitalWrite(13, HIGH);
}

void loop(){
}
```

## 存储各类型数据到EEPROM

``` c
/*
OpenJumper Examples
写入float类型到EEPROM
奈何col  2013.2.2
www.openjumper.com
*/

#include <EEPROM.h>
union data {
    float a;
    byte b[4];
};
data c;
int addr = 0;
int led = 13;

void setup()
{
    c.a = 987.65;
    for (int i = 0; i < 4; i++)
        EEPROM.write(i, c.b[i]);
    pinMode(led, OUTPUT);
}

void loop()
{
    //LED闪烁，提示任务已完成
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
}

/*
OpenJumper Examples
从EEPROM读出float类型
奈何col  2013.2.2
www.openjumper.com
*/

#include <EEPROM.h>
union data
{
  float a;
  byte b[4];
};
data c;
int addr = 0;
int led = 13;

void setup(){
  for(int i=0;i<4;i++)
  c.b[i]=EEPROM.read(i);
  Serial.begin(9600);     
}

void loop(){
  //输出
  Serial.println(c.a);
  delay(1000);    
}
```

## SD创建文件

``` c
#include <SD.h>

File myFile;

void setup()
{
    //我们在2号引脚上连接一个按键模块，用以控制程序开始
    pinMode(2, INPUT_PULLUP);
    while (digitalRead(2))
    {
    }

    // 初始化串口通信
    Serial.begin(9600);
    Serial.print("Initializing SD card...");

    // Arduino上的SS引脚(UNO的10号引脚， Mega的53号引脚)
    // 必须保持在输出模式，否则SD库无法工作
    pinMode(10, OUTPUT);
    if (!SD.begin(4))
    {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");

    if (SD.exists("arduino.txt"))
    {
        Serial.println("arduino.txt exists.");
    }
    else
    {
        Serial.println("arduino.txt doesn't exist.");
    }

    // 打开一个新文件，并立即关闭。
    // 如果指定文件不存在，将用该名称创建一个文件
    Serial.println("Creating arduino.txt...");
    SD.open("arduino.txt", FILE_WRITE);
    myFile.close();

    // 检查文件是否存在
    if (SD.exists("arduino.txt"))
    {
        Serial.println("arduino.txt exists.");
    }
    else
    {
        Serial.println("arduino.txt doesn't exist.");
    }
}

void loop()
{
    // 该程序只运行一次，所以在loop中没有其他操作
}
```

## SD删除文件

```c 
#include <SD.h>

File myFile;

void setup()
{
    //我们在2号引脚上连接一个按键模块，用以控制程序开始
    pinMode(2, INPUT_PULLUP);
    while (digitalRead(2))
    {
    }

    // 初始化串口通信
    Serial.begin(9600);
    Serial.print("Initializing SD card...");

    // Arduino上的SS引脚(UNO的10号引脚， Mega的53号引脚)
    // 必须保持在输出模式否则SD库无法工作
    pinMode(10, OUTPUT);
    if (!SD.begin(4))
    {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");

    if (SD.exists("arduino.txt"))
    {
        Serial.println("arduino.txt exists.");
    }
    else
    {
        Serial.println("arduino.txt doesn't exist.");
    }

    // 删除文件
    Serial.println("Removing arduino.txt...");
    SD.remove("arduino.txt");

    // 检查文件是否存在
    if (SD.exists("arduino.txt"))
    {
        Serial.println("arduino.txt exists.");
    }
    else
    {
        Serial.println("arduino.txt doesn't exist.");
    }
}

void loop()
{
    // 该程序只运行一次，所以在loop中没有其他操作
}
```

## SD写文件

``` c
#include <SD.h>

File myFile;

void setup()
{
  //我们在2号引脚上连接一个按键模块，用以控制程序开始
  pinMode(2,INPUT_PULLUP);
  while(digitalRead(2)){}

// 初始化串口通信
  Serial.begin(9600);
  while (!Serial) {
    ; // 等待串口连接，该方法只适用于Leonardo
  }
Serial.print("Initializing SD card...");

  // Arduino上的SS引脚(UNO的10号引脚， Mega的53号引脚) 
  // 必须保持在输出模式否则SD库无法工作
  pinMode(10, OUTPUT);

  if (!SD.begin(4)) {
Serial.println("initialization failed!");
    return;
  }
Serial.println("initialization done.");

  // 打开文件，需要注意的是一次只能打开一个文件
  // 如果你要打开另一个文件，必须先关闭之前打开的文件
  myFile = SD.open("arduino.txt", FILE_WRITE);

  // 如果文件打开正常，那么开始写文件
  if (myFile) {
Serial.print("Writing to arduino.txt...");
myFile.println("Hello Arduino!");

	// 关闭这个文件
    myFile.close();
Serial.println("done.");
  } 
else {
    // 如果文件没有被正常打开，那么输出错误提示
Serial.println("error opening arduino.txt ");
  }
}

void loop()
{
	//程序只运行一次，因此loop中没有其他操作
}
```

## SD读文件

```c
#include <SD.h>

File myFile;

void setup(){
  //我们在2号引脚上连接一个按键模块，用以控制程序开始
  pinMode(2,INPUT_PULLUP);
  while(digitalRead(2)){}

// 初始化串口通信
  Serial.begin(9600);
  while (!Serial) {
    ; // 等待串口连接，该方法只适用于Leonardo
  }
Serial.print("Initializing SD card...");

  // Arduino上的SS引脚(UNO的10号引脚， Mega的53号引脚) 
  // 必须保持在输出模式否则SD库无法工作
  pinMode(10, OUTPUT);

  if (!SD.begin(4)) {
Serial.println("initialization failed!");
    return;
  }
Serial.println("initialization done.");

  // 打开文件，需要注意的是，一次只能打开一个文件
  // 如果你要打开另一个文件，必须先关闭之前打开的文件
  myFile = SD.open("arduino.txt");

  // 如果文件打开正常，那么开始读文件
  if (myFile) {
    Serial.println("arduino.txt:");

    // 读取文件数据，直到文件结束
    while (myFile.available()) {
	Serial.write(myFile.read());
    }
    // 关闭这个文件
    myFile.close();
} 
else {
	//如果文件没有正常打开，那么输出错误提示
Serial.println("error opening arduino.txt ");
}
}

void loop(){
	//程序只运行一次，因此loop中没有其他操作
}
```

## SD卡环境数据记录器

物料清单：DHT11数字温湿度模块 Dht11.read(pin);(需要DHT11类库) 引脚VCC DATA NC(悬空) GND

```c
#include <dht11.h>

dht11 DHT11;

#define DHT11PIN 2

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    Serial.println("\n");
    // 读取传感器数据
    int chk = DHT11.read(DHT11PIN);
    Serial.print("Read sensor: ");
    // 检测数据是否正常接收
    switch (chk)
    {
    case DHTLIB_OK:
        Serial.println("OK");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        Serial.println("Checksum error");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.println("Time out error");
        break;
    default:
        Serial.println("Unknown error");
        break;
    }
    // 输出湿度与温度信息
    Serial.print("Humidity (%): ");
    Serial.println(DHT11.humidity);
    Serial.print("Temperature (oC): ");
    Serial.println(DHT11.temperature);

    delay(1000);
}
```

## 温湿度记录器

物料清单：光敏模块、DHT11模块、TF卡模块 analogRead()函数和DHT11类库采集光线和温湿度数据，并写入SD卡

```c
#include <SD.h>
#include <dht11.h>

dht11 DHT11;
#define DHT11_PIN 2   // DHT11引脚
#define LIGHT_PIN A0// 光敏引脚
const int chipSelect = 4;// TF卡CS选择引脚

void setup()
{
	// 初始化串口
	Serial.begin(9600);
    //将SS引脚设置为输出状态，UNO为10号引脚
	pinMode(10, OUTPUT);
	// 初始化SD卡
	Serial.println("Initializing SD card");
	if (!SD.begin(chipSelect))
	{
		Serial.println("initialization failed!");
		return;
	}
	Serial.println("initialization done.");
}

void loop()
{
	// 读取DHT11的数据
	Serial.println("Read data from DHT11");  
	DHT11.read(DHT11_PIN);

	// 读取光敏模块数据
	Serial.println("Read data from Light Sensor");  
	int light=analogRead(LIGHT_PIN);

	// 打开文件并将DHT11检测到的数据写入文件
	Serial.println("Open file and write data");  
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	if (dataFile)
	{
		dataFile.print(DHT11.humidity);
		dataFile.print(",");
		dataFile.print(DHT11.temperature);
		dataFile.print(",");
		dataFile.println(light);
		dataFile.close();
	} 
	else
	{
		Serial.println("error opening datalog.txt");
	} 

	//延时一分钟
	Serial.println("Wait for next loop"); 
	delay(60000);
}
```

## 红外接收

```c
/*
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>

int RECV_PIN = 11;// 红外一体化接收头连接到Arduino 11号引脚

IRrecv irrecv(RECV_PIN);

decode_results results;// 用于存储编码结果的对象

void setup()
{
  Serial.begin(9600);// 初始化串口通信
  irrecv.enableIRIn();// 初始化红外解码
}

void loop() {
  if (irrecv.decode(&results))
{
    Serial.println(results.value, HEX);
    irrecv.resume(); // 接收下一个编码
  }
}
```

## 红外发射

```c
/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>

IRsend irsend;

void setup()
{
  Serial.begin(9600); // 初始化串口通信
}

void loop() {
  if (Serial.read() != -1) {
    for (int i = 0; i < 3; i++) {
      irsend.sendSony(0xa90, 12); // 发送索尼电视机电源开关对应的编码
      delay(40);
    }
  }
}
```

## 遥控家电设备

物料清单：红外一体接收模块、红外发送模块、可红外遥控的家电及遥控器

```c
/*
获取原始红外信号
OpenJumper38K红外一体化接收模块
2013.4.24奈何col
*/

#include <IRremote.h>

int RECV_PIN = 11; //红外接收模块连接在11脚
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup()
{
    Serial.begin(9600);
    irrecv.enableIRIn();
}

void loop()
{
    if (irrecv.decode(&results))
    {
        dump(&results);
        irrecv.resume();
    }
}

void dump(decode_results *results)
{
    int count = results->rawlen;
    Serial.print("Raw (");
    Serial.print(count);
    Serial.print("): ");

    for (int i = 0; i < count; i++)
    {
        Serial.print(results->rawbuf[i] * USECPERTICK);
        Serial.print(",");
    }
    Serial.println();
}
```

## LCD1602的hello world

```c
/*
LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCDand shows the time.
*/

// 包含头文件
#include <LiquidCrystal.h>

// 实例化一个名为lcd的LiquidCrysta类型对象，并初始化相关引脚
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  // 设置LCD有几列几行，1602LCD为16列2行
  lcd.begin(16, 2);
  //打印一段信息到LCD上
lcd.print("hello, world!");
}

void loop() {
  // 将光标设置在列 0, 行 1
  // 注意：在1602 LCD上行列的标号都是从0开始
  lcd.setCursor(0, 1);
  // 将系统运行时间打印到LCD上
  lcd.print(millis()/1000);
}
```

## 将串口输入数据显示到LCD1602上

```c
/*
  LiquidCrystal Library - Serial Input

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
can usually tell them by the 16-pin interface.

 This sketch displays text sent over the serial port
(e.g. from the Serial Monitor) on an attached LCD.
 */

// 包含头文件
#include <LiquidCrystal.h>

//实例化一个名为lcd的LiquidCrysta类型对象，并初始化相关引脚
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup(){
    //设置LCD有几列几行，1602LCD为16列2行
  lcd.begin(16, 2);
  // 初始化串口通信功能
  Serial.begin(9600);
}

void loop()
{
  // 当串口接收到字符时…
  if (Serial.available()) {
    // 等待所有数据输入进缓冲区
    delay(100);
    // 清屏
    lcd.clear();
    // 读取所有可用的字符
    while (Serial.available() > 0) {
      // 将字符一个一个的输出到LCD上。
      lcd.write(Serial.read());
    }
  }
}
```

## LCD1602显示滚动效果

```c
/*
  LiquidCrystal Library - scrollDisplayLeft() and scrollDisplayRight()

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD and uses the
 scrollDisplayLeft() and scrollDisplayRight() methods to scroll
the text.
*/

//包含头文件
#include <LiquidCrystal.h>

//实例化一个名为lcd的LiquidCrysta类型对象，并初始化相关引脚
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup()
{
    //设置LCD有几列几行，1602LCD为16列2行
    lcd.begin(16, 2);
    // 打印一段信息到LCD上
    lcd.print("hello, world!");
    delay(1000);
}

void loop()
{
    // 向左滚动13格
    // 移动到显示区域以外
    for (int positionCounter = 0; positionCounter < 13; positionCounter++)
    {
        // 向左移动一格
        lcd.scrollDisplayLeft();
        // 等待一会儿
        delay(150);
    }

    // 向右滚动29格
    //移动到显示区域以外
    for (int positionCounter = 0; positionCounter < 29; positionCounter++)
    {
        // 向右移动一格
        lcd.scrollDisplayRight();
        //等待一会儿
        delay(150);
    }

    // 向左滚动16格
    // 移回中间位置
    for (int positionCounter = 0; positionCounter < 16; positionCounter++)
    {
        // 向左移动一格
        lcd.scrollDisplayLeft();
        //等待一会儿
        delay(150);
    }

    // 每次循环结束后，等待一会儿，再开始下一次循环
    delay(1000);
}
```

## LCD1602显示自定义字符

```c
/*
  LiquidCrystal Library - Custom Characters

 Demonstrates how to add custom characters on an LCD  display.  
 The LiquidCrystal library works with all LCD displays that are 
compatible with the  Hitachi HD44780 driver. There are many of 
them out there, and you can usually tell them by the 16-pin interface.

 This sketch prints "I <heart> Arduino!" and a little dancing man
to the LCD.
*/

// 包含头文件
#include <LiquidCrystal.h>

//实例化一个名为lcd的LiquidCrysta类型对象，并初始化相关引脚
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// 自定义字符：
byte heart[8] = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000};

byte smiley[8] = {
    0b00000,
    0b00000,
    0b01010,
    0b00000,
    0b00000,
    0b10001,
    0b01110,
    0b00000};

byte frownie[8] = {
    0b00000,
    0b00000,
    0b01010,
    0b00000,
    0b00000,
    0b00000,
    0b01110,
    0b10001};

byte armsDown[8] = {
    0b00100,
    0b01010,
    0b00100,
    0b00100,
    0b01110,
    0b10101,
    0b00100,
    0b01010};

byte armsUp[8] = {
    0b00100,
    0b01010,
    0b00100,
    0b10101,
    0b01110,
    0b00100,
    0b00100,
    0b01010};
void setup()
{
    // 创建一个新字符
    lcd.createChar(8, heart);
    // 创建一个新字符
    lcd.createChar(1, smiley);
    // 创建一个新字符
    lcd.createChar(2, frownie);
    // 创建一个新字符
    lcd.createChar(3, armsDown);
    // 创建一个新字符
    lcd.createChar(4, armsUp);

    //设置LCD有几列几行，1602LCD为16列2行
    lcd.begin(16, 2);
    // 输出信息到LCD上
    lcd.print("I ");
    lcd.write(8);
    lcd.print(" Arduino! ");
    lcd.write(1);
}

void loop()
{
    // 读A0引脚连接的电位器的值
    int sensorReading = analogRead(A0);
    // 将数据范围转换到 200 - 1000:
    int delayTime = map(sensorReading, 0, 1023, 200, 1000);
    // 设置光标在第二行，第五列
    lcd.setCursor(4, 1);
    // 小人手臂放下
    lcd.write(3);
    delay(delayTime);
    lcd.setCursor(4, 1);
    // 小人手臂抬起
    lcd.write(4);
    delay(delayTime);
}
```

## 制作电子时钟

物料清单：LCD1602、DS1307时钟模块、Arduino Time库、DS1307RTC库

```c
/*
设置DS1307的时间
RTC模块的使用
*/

//声明这个模块用到的三个类库
#include <DS1307RTC.h>
#include <Time.h>
#include <Wire.h>

const char *monthName[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// tmElements_t为保存日期时间的结构体类型
tmElements_t tm;

void setup()
{
    bool parse = false;
    bool config = false;

    // 获取编译时的时间和日期
    if (getDate(__DATE__) && getTime(__TIME__))
    {
        parse = true;
        // 将时间数据写入RTC模块
        if (RTC.write(tm))
        {
            config = true;
        }
    }

    Serial.begin(9600);
    delay(200);
    if (parse && config)
    {
        Serial.print("DS1307 configured Time=");
        Serial.print(__TIME__);
        Serial.print(", Date=");
        Serial.println(__DATE__);
    }
    else if (parse)
    {
        Serial.println("DS1307 Communication Error :-{");
        Serial.println("Please check your circuitry");
    }
    else
    {
        Serial.print("Could not parse info from the compiler, Time=\"");
        Serial.print(__TIME__);
        Serial.print("\", Date=\"");
        Serial.print(__DATE__);
        Serial.println("\"");
    }
}

void loop()
{
}
// 获取时间数据并存入tm
bool getTime(const char *str)
{
    int Hour, Min, Sec;

    if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3)
        return false;
    tm.Hour = Hour;
    tm.Minute = Min;
    tm.Second = Sec;
    return true;
}
// 获取日期数据并存入tm
bool getDate(const char *str)
{
    char Month[12];
    int Day, Year;
    uint8_t monthIndex;

    if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3)
        return false;
    for (monthIndex = 0; monthIndex < 12; monthIndex++)
    {
        if (strcmp(Month, monthName[monthIndex]) == 0)
            break;
    }
    if (monthIndex >= 12)
        return false;
    tm.Day = Day;
    tm.Month = monthIndex + 1;
    tm.Year = CalendarYrToTm(Year);
    return true;
}


#include <DS1307RTC.h>
#include <Time.h>
#include <Wire.h>

void setup()
{
    Serial.begin(9600);
    delay(200);
    Serial.println("DS1307RTC Read Test");
    Serial.println("-------------------");
}

void loop()
{
    tmElements_t tm;
    // 读出DS1307中的时间数据
    if (RTC.read(tm))
    {
        Serial.print("Ok, Time = ");
        print2digits(tm.Hour);
        Serial.write(':');
        print2digits(tm.Minute);
        Serial.write(':');
        print2digits(tm.Second);
        Serial.print(", Date (D/M/Y) = ");
        Serial.print(tm.Day);
        Serial.write('/');
        Serial.print(tm.Month);
        Serial.write('/');
        Serial.print(tmYearToCalendar(tm.Year));
        Serial.println();
    }
    else
    {
        if (RTC.chipPresent())
        {
            Serial.println("The DS1307 is stopped.  Please run the SetTime");
            Serial.println("example to initialize the time and begin running.");
            Serial.println();
        }
        else
        {
            Serial.println("DS1307 read error!  Please check the circuitry.");
            Serial.println();
        }
        delay(9000);
    }
    delay(1000);
}

void print2digits(int number)
{
    if (number >= 0 && number < 10)
    {
        Serial.write('0');
    }
    Serial.print(number);
}

#include <DS1307RTC.h>
#include <Time.h>
#include <Wire.h>

void setup()
{
    Serial.begin(9600);
    delay(200);
    Serial.println("DS1307RTC Read Test");
    Serial.println("-------------------");
}

void loop()
{
    tmElements_t tm;
    // 读出DS1307中的时间数据
    if (RTC.read(tm))
    {
        Serial.print("Ok, Time = ");
        print2digits(tm.Hour);
        Serial.write(':');
        print2digits(tm.Minute);
        Serial.write(':');
        print2digits(tm.Second);
        Serial.print(", Date (D/M/Y) = ");
        Serial.print(tm.Day);
        Serial.write('/');
        Serial.print(tm.Month);
        Serial.write('/');
        Serial.print(tmYearToCalendar(tm.Year));
        Serial.println();
    }
    else
    {
        if (RTC.chipPresent())
        {
            Serial.println("The DS1307 is stopped.  Please run the SetTime");
            Serial.println("example to initialize the time and begin running.");
            Serial.println();
        }
        else
        {
            Serial.println("DS1307 read error!  Please check the circuitry.");
            Serial.println();
        }
        delay(9000);
    }
    delay(1000);
}

void print2digits(int number)
{
    if (number >= 0 && number < 10)
    {
        Serial.write('0');
    }
    Serial.print(number);
}
```

```c
/*
DS1307和1602 LCD制作电子时钟
*/
// 引用会使用到的四个库文件
#include <DS1307RTC.h>
#include <Time.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// 实例化一个名为lcd的LiquidCrysta类型对象，并初始化相关引脚
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup()
{
    // 设置LCD有几列几行，1602LCD为16列2行
    lcd.begin(16, 2);
    // 打印一段信息到LCD上
    lcd.print("This is a Clock");
    delay(3000);
    lcd.clear();
}

void loop()
{
    tmElements_t tm;
    // 读出DS1307中的时间数据，并存入tm中
    if (RTC.read(tm))
    {
        // 清除屏幕显示内容
        lcd.clear();
        //在LCD第一行输出日期信息
        lcd.setCursor(0, 0);
        lcd.print(tmYearToCalendar(tm.Year));
        lcd.print("-");
        lcd.print(tm.Month);
        lcd.print("-");
        lcd.print(tm.Day);
        //在LCD第二行输出时间信息
        lcd.setCursor(8, 1);
        lcd.print(tm.Hour);
        lcd.print(":");
        lcd.print(tm.Minute);
        lcd.print(":");
        lcd.print(tm.Second);
    }
    // 如果读取数据失败，则输出错误提示
    else
    {
        lcd.setCursor(0, 1);
        lcd.print("error");
    }
    //每秒钟更新一次显示内容
    delay(1000);
}
```

## u8glib纯文本显示"Hello Arduino"

```c
/*
使用u8glib显示字符串
图形显示器：OpenJumper MINI 12864
控制器：Arduino UNO
*/

// 包含头文件，并新建u8g对象
#include "U8glib.h"
U8GLIB_MINI12864 u8g(10, 9, 8);

// draw函数用于包含实现显示内容的语句
void draw()
{
    // 设置字体
    u8g.setFont(u8g_font_unifont);
    // 设置文字及其显示位置
    u8g.drawStr(0, 20, "Hello Arduino");
}

void setup()
{
}

void loop()
{
    // u8glib图片循环结构：
    u8g.firstPage();
    do
    {
        draw();
    } while (u8g.nextPage());
    // 等待一定时间后重绘
    delay(500);
}
```

## u8glib数据显示

```c
/*
使用print函数输出数据到LCD
图形显示器：OpenJumper MINI 12864
控制器：Arduino UNO
*/

#include "U8glib.h"

U8GLIB_MINI12864 u8g(10, 9, 8);

String t1 = "OpenJumper";
String t2 = "MINI";
int t3 = 12864;

// draw函数用于包含实现显示内容的语句
void draw(void)
{
    // 设定字体->指定输出位置->输出数据
    u8g.setFont(u8g_font_ncenB14);
    u8g.setPrintPos(0, 20);
    u8g.print(t1);
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(50, 40);
    u8g.print(t2);
    u8g.setPrintPos(45, 60);
    u8g.print(t3);
}

void setup(void)
{
}

void loop(void)
{
    // u8glib图片循环结构：
    u8g.firstPage();
    do
    {
        draw();
    } while (u8g.nextPage());
    // 等待一定时间后重绘
    delay(500);
}
```

## u8glib显示图片

```c
/*
使用u8glib显示位图
图形显示器：OpenJumper MINI 12864
控制器：Arduino UNO
*/

#include "U8glib.h"
U8GLIB_MINI12864 u8g(10, 9, 8);
/*宽度x高度=96x64*/
#define width 96
#define height 64

static unsigned charbitmap[] U8G_PROGMEM = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x80,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0xE0,0xFF,0xFF,0x1F,
0x00,0x78,0x00,0x00,0x00,0xF0,0x1F,0x00,0x7C,0x00,0x00,0xFC,0xC0,0x7F,0x00,0x00,
//为节约篇幅，此处代码省略
0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,
0xFF,0xFF,0xFF,0xFF,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFF,0xFF,0xFF,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};

void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.drawXBMP( 0, 0, width, height, bitmap);
}

void setup(void) {
}

void loop(void) 
{
  u8g.firstPage();  
  do
{
draw();
  } while( u8g.nextPage() );
  // 延时一定时间，再重绘图片
  delay(500);
}
```

