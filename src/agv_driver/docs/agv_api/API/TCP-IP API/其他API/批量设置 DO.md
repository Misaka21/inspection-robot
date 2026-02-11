![...](..\..\TCP-IP%20API\其他API\images\批量设置%20DO.001.png)

**批量设置 DO**


```<p>注意：</p><p>该 API 用于批量设置机器人控制器上的 DO (Digital Output) 状态。</p>```

**请求**

- 编号: 6002 (0x1772)
- 名称: robot\_other\_setdos\_req
- 描述: 批量设置 DO
- JSON 数据区: 见下


```JSON
// 数据区必须为 JSON array, array 中的每个对象与 6001 指令中相同
// 以下为同时设置 DO 0,1,2,3 的状态，分别设置为高、高、低、低
[
  {"id":0,"status":true},
  {"id":1,"status":true},
  {"id":2,"status":false},
  {"id":3,"status":false}
]```

**请求示例**


```JSON
[
  {"id":0,"status":true},
  {"id":1,"status":true},
  {"id":2,"status":false},
  {"id":3,"status":false},
  {"id":4,"status":true},
  {"id":5,"status":true},
  {"id":6,"status":false}
]```

**响应**

- 编号: 16002 (0x3E82)
- 名称: robot\_other\_setdos\_res
- 描述: 批量设置 DO 的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "create\_on": "2024-05-07T13:54:38.796+0800",
  "ret\_code": 0
}```


