![...](..\..\TCP-IP%20API\机器人配置API\images\重置%20GNSS%20硬件配置.001.png)

**重置 GNSS 硬件配置**

**请求**

- 编号: 4460 (0x116c)
- 名称: robot\_config\_reset\_gnss\_req
- 描述: 重置 GNSS 硬件配置
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|id|string|GNSS id|是|

**请求示例**


```Plaintext
// 重置id为g1的GNSS硬件配置
// {"id":"g1"}
5A 01 00 00 00 00 00 0B 11 6C 11 6C 00 00 00 00
7B 22 69 64 22 3A 22 67 31 22 7D```

**响应**

- 编号: 14460 (0x387c)
- 名称: robot\_config\_reset\_gnss\_res
- 描述: 重置 GNSS 硬件配置的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|status|bool|重置状态：重置失败=false；重置成功=true|是|
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


