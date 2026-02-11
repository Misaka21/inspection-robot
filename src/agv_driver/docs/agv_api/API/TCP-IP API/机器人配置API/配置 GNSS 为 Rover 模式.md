![...](..\..\TCP-IP%20API\机器人配置API\images\配置%20GNSS%20为%20Rover%20模式.001.png)

**配置 GNSS 为 Rover 模式**

**请求**

- 编号: 4462 (0x116e)
- 名称: robot\_config\_set\_gnss\_rover\_req
- 描述: 配置 GNSS 为 Rover 模式
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|id|string|GNSS id|是|

**请求示例**


```Plaintext
// {"id":"g1"}
5A 01 00 00 00 00 00 0B 11 6C 11 6E 00 00 00 00
7B 22 69 64 22 3A 22 67 31 22 7D```

**响应**

- 编号: 14462 (0x387e)
- 名称: robot\_config\_set\_gnss\_rover\_res
- 描述: 配置 GNSS 为 Rover 模式的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|status|bool|配置状态：配置失败=false；配置成功=true|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "create\_on": "2022-01-07T14:10:39.629Z",
  "ret\_code": 0,
  "status": false
}```


