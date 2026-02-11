![...](..\..\TCP-IP%20API\机器人导航API\images\根据任务id清除指定导航路径.001.png)

**根据任务id清除指定导航路径**

3.3.5.76 < rbk

```根据 task\_id 清除指定路径导航 [3066 ](指定路径导航.md)所下发的站点序列。 task\_id 之后（不含 task\_id ）的任务都会被取消，机器人会执行完 task\_id 的移动任务，但不会执行 task\_id 的点位动作。```
**请求**

- 编号: 3068 (0x0BFC)
- 名称: robot\_task\_safeclearmovements\_req
- 描述: 根据任务id清除指定导航路径
- JSON 数据区: 见下表

```**字段名**|类型|描述|可缺省``` :- | :- | :- |
|task\_id|string|3066 api 中的 task\_id 任务号|否|


```需注意：如果task\_id为空或者不存在，机器人将不作反应。```

**请求示例**

```JSON
{
  "task\_id": "12344321"
}```

**响应**

- 编号: 13068 (0x330C)
- 名称: robot\_task\_safeclearmovements\_res
- 描述: 根据任务id清除指定导航路径的响应
- JSON 数据区: 见下表

```字段名|类型|描述|可缺省``` :- | :- | :- |
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|

**响应示例**


```JSON
{
  "ret\_code": 0
}```


```Plaintext
5A 01 00 01 00 00 00 00 0B FB 00 00 00 00 00 00
7B 22 72 65 74 5F 63 6F 64 65 22 3A 30 7D```


