![...](..\..\TCP-IP%20API\其他API\images\设置%20DO.001.png)

**设置 DO**


```<p>注意：</p><p>该 API 用于设置机器人控制器上的 DO (Digital Output) 状态。</p>```

**请求**

- 编号: 6001 (0x1771)
- 名称: robot\_other\_setdo\_req
- 描述: 设置 DO
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|id|number|DO 的 id|否|
|status|boolean|true 为高电平, false 为低电平|否|

**请求示例**


```JSON
5A 01 00 01 00 00 00 17 17 71 00 00 00 00 00 00
{"id":3,"status":false}
// 序列化后十六进制为：
7B 22 69 64 22 3A 33 2C 22 73 74 61 74 75 73 22 
3A 66 61 6C 73 65 7D```

**响应**

- 编号: 16001 (0x3E81)
- 名称: robot\_other\_setdo\_res
- 描述: 设置 DO 的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "create\_on":"2024-05-07T10:40:09.672+0800",
  "ret\_code":0
}```


