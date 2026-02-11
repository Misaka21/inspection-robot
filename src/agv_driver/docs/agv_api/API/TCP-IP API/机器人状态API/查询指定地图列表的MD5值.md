![...](..\..\TCP-IP%20API\机器人状态API\images\查询指定地图列表的MD5值.001.png)

**查询指定地图列表的MD5值**

**请求**

- 编号: 1302 (0x0516)
- 名称: robot\_status\_mapmd5\_req
- 描述: 查询指定地图列表的MD5值
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|map\_names|array[string]|需要查询的地图列表，应该保证机器人中存在地图列表中所有的地图|否|

**请求示例**


```JSON
// {'map\_names': ["default.smap"]}
5A 01 00 01 00 00 00 1F 05 16 00 00 00 00 00 00 
7B 22 6D 61 70 5F 6E 61 6D 65 73 22 3A 20 5B 22 
64 65 66 61 75 6C 74 2E 73 6D 61 70 22 5D 7D```

**响应**

- 编号: 11302 (0x2C26)
- 名称: robot\_status\_mapmd5\_res
- 描述: 查询指定地图列表的MD5值的响应
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|map\_info|array[object]|地图的MD5值和名字，数据示例见下文|是|
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "create\_on": "2021-04-20T17:20:58.704Z",
  "map\_info": [
    {
      "md5": "9cffdb59dd1f7adc4c41fb425f4abdb2",
      "name": "default.smap"
    }
  ],
  "ret\_code": 0
}```


