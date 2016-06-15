附件里面 ESC32keil  是 一个 电调驱动板， ADC采样和 PWM 输入采样可以参考他的
ServoCTL-0C-Sch 是线路图
1. 1路PWM输入
2. 4路IO 输出
3. 2路模拟量输入，用于过流检测

我估计后面的工作流程要改的。
上电默认状态，LED 慢闪（暂时可以不加）
上电时候如果  PWM 在 1200 以下的时候 ， M1_OUT1 、M2_OUT1 输出 High， M1_OUT2 、M2_OUT2输出Low。
此时 MCU 监测 M1_ISense  和M2_ISense 这2个信号，
这个ADC 是10bit 的， 当监测到 采样回来的数据大于512 且持续1秒，就可以让 M1_OUT1 或者M2_OUT1 输出 Low。
    
反之如果
PWM 在 1700 以上的时候 ， M1_OUT1 、M2_OUT1 输出 Low， M1_OUT2 、M2_OUT2输出High
此时 MCU 监测 M1_ISense  和M2_ISense 这2个信号，
这个ADC 是10bit 的， 当监测到 采样回来的数据大于512 且持续1秒，就可以让 M1_OUT2 或者M2_OUT2 输出 Low。

另外：
    当 M1_OUT1 、M2_OUT1 和 M1_OUT2 、M2_OUT2 这4个信号 只要有一组 处于High 的时候，内部就要有 countor 来计算 high  的时间。
    这个 high 的时间 和 读取回来 Sens 的值 都需要预留可以设置的 宏定义。
	
	
描述：
pwm 在1200以下为关闭指令，1700以上为打开指令
1.第一次开机时当收到pwm命令时做关闭动作，当关闭动作完成（time out/current over flow）才能响应pwm 输入
2.每次收到pwm输入与当前脚架状态不一致时做动作
3.每次收到pwm输入时延时0.5s做动作。
4.当有过流时，停止该路脚架动作，当两路都过流时则改变状态为打开或关闭
5.打开或关闭动作超时时则改变状态为已打开或已关闭。