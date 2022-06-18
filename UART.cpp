#include "stdafx.h"

#include "Form1.h"

 

#include <windows.h>  //添加头文件

 

 

#include <tchar.h>

 

using namespace commtest;

 

bool openport(char *portname);

 

[STAThreadAttribute]

int main(array<System::String ^> ^args)

{

bool open;

open=openport("com2"); //打开串口2

// 在创建任何控件之前启用 Windows XP 可视化效果

Application::EnableVisualStyles();

Application::SetCompatibleTextRenderingDefault(false); 

 

// 创建主窗口并运行它

Application::Run(gcnew Form1());

 

return 0;

}

 

bool openport(char *portname)//打开串口

{

HANDLE hComm;

hComm = CreateFile(portname, //串口号

 GENERIC_READ | GENERIC_WRITE, //允许读写

 0, //通讯设备必须以独占方式打开

 0, //无安全属性

 OPEN_EXISTING, //通讯设备已存在

 FILE_FLAG_OVERLAPPED, //异步I/O

 0); //通讯设备不能用模板打开

if (hComm == INVALID_HANDLE_VALUE)

{

CloseHandle(hComm);

return FALSE;

}

else

return true;

}
