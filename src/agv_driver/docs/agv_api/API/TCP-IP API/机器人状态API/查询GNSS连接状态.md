![...](..\..\TCP-IP%20API\机器人状态API\images\查询GNSS连接状态.001.png)

**查询GNSS连接状态**

**请求**

- 编号: 1760 (0x06e0)
- 名称: robot\_status\_gnsscheck\_req
- 描述: 查询机器人 GNSS 连接状态
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|id|string|GNSS id|是|

**请求示例**


```Plaintext
// 查询id为g1的GNSS连接状态
// {"id":"g1"}
5A 01 00 00 00 00 00 0B 06 E0 06 E0 00 00 00 00
7B 22 69 64 22 3A 22 67 31 22 7D```

**响应**

- 编号: 11760 (0x2DF0)
- 名称: robot\_status\_gnsscheck\_res
- 描述: 查询机器人 GNSS 连接状态的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|alive|bool|GNSS 连接状态：未连接=false；已连接=true|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
    "alive": false,
    "create\_on": "2022-01-06T18:02:45.960Z",
    "ret\_code": 0
}```


