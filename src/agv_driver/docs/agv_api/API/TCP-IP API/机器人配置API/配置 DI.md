![...](..\..\TCP-IP%20API\机器人配置API\images\配置%20DI.001.png)

**配置 DI**

**请求**

- 编号: 4140 (0x102C)
- 名称: robot\_config\_DI\_req
- 描述: 配置 DI
- JSON 数据区: 见下表

```<p>注意：</p><p>用于配置某个 id 的 DI 是否启用。</p>```


```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|id|number|DI 的 id|否|
|valid|boolean|是否启用, false = 禁用, true = 启用|否|

**请求示例**


```JSON
{
  "id":1,
  "valid":true 
}```

**响应**

- 编号: 14140 (0x373C)
- 名称: robot\_config\_DI\_res
- 描述: 配置 DI的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "create\_on":"2024-05-07T13:34:59.749+0800",
  "ret\_code":0
}```


