![...](..\..\TCP-IP%20API\机器人状态API\images\查询GNSS设备列表.001.png)

**查询GNSS设备列表**

**请求**

- 编号: 1761 (0x06E1)
- 名称: robot\_status\_gnss\_list\_req
- 描述: 查询机器人 GNSS 设备列表
- JSON 数据区: 无

**请求示例**


```Plaintext
5A 01 00 00 00 00 00 00 06 E1 06 E1 00 00 00 00 ```

**响应**

- 编号: 11761 (0x2DF1)
- 名称: robot\_status\_gnss\_list\_res
- 描述: 查询机器人 GNSS 设备列表的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|list|string array|GNSS 设备列表|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
    "list": ["a","b","c","d"],
    "create\_on": "2022-01-06T18:02:45.960Z",
    "ret\_code": 0
}```


