![...](..\..\TCP-IP%20API\机器人状态API\images\查询二维码数据(PGV).001.png)

**查询二维码数据(PGV)**


```PGV 为位置导引视觉系统(Position Guided Vision)```

**请求**

- 编号: 1017 (0x03F9)
- 名称: robot\_status\_pgv\_req
- 描述: 查询二维码数据(PGV)数据
- JSON 数据区: 无

**请求示例**


```Plaintext
5A 01 00 01 00 00 00 00 03 F9 00 00 00 00 00 00```

**响应**

- 编号: 11017 (0x2B09)
- 名称: robot\_status\_pgv\_res
- 描述: 查询二维码数据(PGV)数据的响应
- JSON 数据区: 见下表

```字段名|类型|描述|可缺省``` :- | :- | :- |
|pgvs|array[object]|二维码识别数据, 数据示例见下文|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

object 形式如下:
主要关注 device\_address（PGV 设备的 id）和 tag\_value（二维码标签的数据）


```JSON
{
    // PGV 设备的 id
    "device\_address": 1,
    "error\_code": 0,
    "header": {
        "data\_nsec": "1750000088331",
        "frame\_id": "",
        "pub\_nsec": "0",
        "seq": "0"
    },
    "is\_DMT\_detected": true,
    "is\_absolute\_X\_pos\_valid": true,
    "pgv\_info": {
        "func": 1,
        "r": 0,
        "x": 0,
        "y": 0,
        "z": 0
    },
    "tag\_diff\_angle": 1744,
    "tag\_diff\_x": 256,
    "tag\_diff\_y": 47,
    // 二维码标签的数据
    "tag\_value": 1,
    "warning\_code": 0
}```

**响应示例**

如下为两个 PGV 设备的数据:

```JSON
{
  "pgvs": [
    {
      "device\_address": 0,
      "error\_code": 0,
      "header": {
        "data\_nsec": "1749997924588",
        "frame\_id": "",
        "pub\_nsec": "0",
        "seq": "0"
      },
      "is\_DMT\_detected": false,
      "is\_absolute\_X\_pos\_valid": false,
      "pgv\_info": {
        "func": 1,
        "r": 0,
        "x": 0,
        "y": 0,
        "z": 0
      },
      "tag\_diff\_angle": 0,
      "tag\_diff\_x": 0,
      "tag\_diff\_y": 0,
      "tag\_value": 0,
      "warning\_code": 0
    },
    {
      "device\_address": 1,
      "error\_code": 0,
      "header": {
        "data\_nsec": "1749997924588",
        "frame\_id": "",
        "pub\_nsec": "0",
        "seq": "0"
      },
      "is\_DMT\_detected": true,
      "is\_absolute\_X\_pos\_valid": true,
      "pgv\_info": {
        "func": 2,
        "r": 0,
        "x": 0,
        "y": 0,
        "z": 0
      },
      "tag\_diff\_angle": 1790,
      "tag\_diff\_x": 54,
      "tag\_diff\_y": 3,
      "tag\_value": 1,
      "warning\_code": 0
    }
  ]
}```

为方便阅读，这里 JSON 数据采用了展开的形式, 实际使用中为减少数据流 JSON 数据区是紧凑的, 不会有多余的空格与换行, 后文中同理。

