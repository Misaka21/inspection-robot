![...](..\..\TCP-IP%20API\其他API\images\写入modbus数据.001.png)

**写入modbus数据**

（3.4.6.14+）

**请求**

- 编号: 6086 (0x17C6)
- 名称: ROBOT\_OTHER\_SET\_MODBUS\_REQ
- 描述: 写入modbus数据
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|4x|json|暂时只处理可写寄存器数据，json内key为需写入的位，value为写入值，目前仅能写入110-199位|否|

**请求示例**



```JSON
{
  "4x": {
  "120":1,
  "121":2
    }
}```

**响应**

- 编号: 16086
- 名称: ROBOT\_OTHER\_SET\_MODBUS\_RES
- 描述: 写入modbus数据的响应
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
5A 01 00 01 00 00 00 0E 3E B1 17 A1 00 00 00 00
7B 22 72 65 74 5F 63 6F 64 65 22 3A 30 7D```


