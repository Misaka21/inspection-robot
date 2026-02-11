![...](..\..\TCP-IP%20API\机器人配置API\images\设置第三方%20Warning.001.png)

**设置第三方 Warning**


```设置后会产生 54900 的 Warning 报警码，Roboshop 可以看到如下报警：
"a warning" 为 msg 中传入的内容。```
![...](..\..\TCP-IP%20API\机器人配置API\images\设置第三方%20Warning.002.png)

**请求**

- 编号: 4802 (0x12C2)
- 名称: robot\_config\_setwarning\_req
- 描述: 设置第三方 Warning
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|msg|string|Warning 的描述，仅限英文|否|

**请求示例**


```JSON
5A 01 00 01 00 00 00 2F 12 C2 00 00 00 00 00 00
{"msg":"a warning from netprotocol tcp/ip api"}
// 序列化后十六进制为：
7B 22 6D 73 67 22 3A 22 61 20 77 61 72 6E 69 6E 
67 20 66 72 6F 6D 20 6E 65 74 70 72 6F 74 6F 63 
6F 6C 20 74 63 70 2F 69 70 20 61 70 69 22 7D```

**响应**

- 编号: 14802 (0x39D2)
- 名称: robot\_config\_setwarning\_res
- 描述: 设置第三方 Warning 的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "ret\_code": 0
}```


```Plaintext
5A 01 00 01 00 00 00 0E 39 D2 12 C2 00 00 00 00
7B 22 72 65 74 5F 63 6F 64 65 22 3A 30 7D```


