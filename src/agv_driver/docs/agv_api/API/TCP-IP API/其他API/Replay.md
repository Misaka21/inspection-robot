![...](..\..\TCP-IP%20API\其他API\images\Replay.001.png)

**Replay**

**请求**

- 编号: 6910 (0x1AFE)
- 名称: robot\_other\_replay\_req
- 描述: Replay插件API
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|api\_type|string|api 类型，"record\_start"表示开始采集，"record\_end"表示停止采集，"replay"表示开始重播，"replay\_end"表示停止重播|否|
|filename|string|当"api\_type"为"replay"时表示需要解析的文件名|是|
|timestamp|string|当"api\_type"为"replay"时表示从该时间戳开始重播，其余情况传空值|是|
**请求示例**


```JSON
{
  "api\_type": "replay", 
  "timestamp": 1678790955854
}```

**响应**

- 编号: 16910 (0x420E)
- 名称: robot\_other\_replay\_res
- 描述: Replay插件API的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|api\_type|string|表示请求的api\_type|否|
|start|string|当请求的"api\_type"为"replay"时，返回文件开头时间戳|是|
|end|string|当请求的"api\_type"为"replay"时，返回文件末尾时间戳|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "api\_type": "record\_start",
  "create\_on": "2022-06-10T14:45:41.849Z",
  "ret\_code": 0,
}```


