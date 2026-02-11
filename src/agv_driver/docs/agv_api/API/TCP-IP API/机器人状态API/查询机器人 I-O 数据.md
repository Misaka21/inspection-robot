![...](..\..\TCP-IP%20API\机器人状态API\images\查询机器人%20I-O%20数据.001.png)

**查询机器人 I/O 数据**

**请求**

- 编号: 1013 (0x03F5)
- 名称: robot\_status\_io\_req
- 描述: 查询机器人 I/O 数据
- JSON 数据区: 无

**请求示例**


```Plaintext
5A 01 00 01 00 00 00 00 03 F5 00 00 00 00 00 00```

**响应**

- 编号: 11013 (0x2B05)
- 名称: robot\_status\_io\_res
- 描述: 查询机器人 I/O 数据的响应
- JSON 数据区: 见下表

```字段名|类型|描述|可缺省``` :- | :- | :- |
|DI|array[Object]|id: 对应的 id 号<br>source: 来源<br>`  `normal = 普通 DI<br>`  `virtual = 虚拟 DI<br>`  `modbus = modbus DI<br>status: 表示高低电平<br>`  `true = 高电平<br>`  `false = 低电平<br>valid: 对应 DI 是否启用<br>`  `true = 启用<br>`  `false = 禁用|是|
|DO|array[Object]|id: 对应的 id 号<br>source: 来源<br>`  `normal = 普通 DO<br>`  `modbus = modbus DO<br>status: 表示高低电平<br>`  `true = 高电平<br>`  `false = 低电平|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "DI": [
    {
      "id": 0,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 1,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 2,
      "source": "normal",
      "status": true,
      "valid": false
    },
    {
      "id": 3,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 4,
      "source": "normal",
      "status": false,
      "valid": true
    },
    {
      "id": 5,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 6,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 7,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 8,
      "source": "normal",
      "status": true,
      "valid": true
    },
    {
      "id": 9,
      "source": "virtual",
      "status": true,
      "valid": true
    }
  ],
  "DO": [
    {
      "id": 0,
      "source": "normal",
      "status": true
    },
    {
      "id": 1,
      "source": "normal",
      "status": true
    },
    {
      "id": 2,
      "source": "normal",
      "status": true
    },
    {
      "id": 3,
      "source": "normal",
      "status": true
    },
    {
      "id": 4,
      "source": "normal",
      "status": true
    },
    {
      "id": 5,
      "source": "normal",
      "status": true
    },
    {
      "id": 6,
      "source": "normal",
      "status": true
    }
  ],
  "ret\_code": 0
}```


