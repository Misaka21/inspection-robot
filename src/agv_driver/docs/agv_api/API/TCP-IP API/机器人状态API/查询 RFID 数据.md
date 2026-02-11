![...](..\..\TCP-IP%20API\机器人状态API\images\查询%20RFID%20数据.001.png)

**查询 RFID 数据**

**请求**

- 编号: 1015 (0x03F7)
- 名称: robot\_status\_rfid\_req
- 描述: 查询 RFID 数据
- JSON 数据区: 无

**请求示例**


```Plaintext
5A 01 00 01 00 00 00 00 03 F7 00 00 00 00 00 00```

**响应**

- 编号: 11015 (0x2B07)
- 名称: robot\_status\_rfid\_res
- 描述: 查询 RFID 数据的响应
- JSON 数据区: 见下表

```字段名|类型|描述|可缺省``` :- | :- | :- |
|rfids|array[int]|扫描到的所有 RFID 标签 id,  如果没扫描到 RFID 标签, 则为空数组|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "ret\_code": 0,
  "rfids": []
}```


