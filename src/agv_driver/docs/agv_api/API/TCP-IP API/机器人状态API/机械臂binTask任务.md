![...](..\..\TCP-IP%20API\机器人状态API\images\机械臂binTask任务.001.png)

**机械臂binTask任务**

**请求**

- 编号: 1671 (0x0687)
- 名称: robot\_status\_armtask\_req
- 描述: 机械臂binTask任务
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|taskId|int|机械臂binTask任务id|否|
|group|string|机械臂binTask任务组名|否|
**请求示例**

```JSON
{
  "taskId": "0",
  "group": "group1"
}```
**响应**

- 编号: 11671 (0x2D97)
- 名称: robot\_status\_armtask\_res
- 描述: 机械臂binTask任务的响应
- JSON 数据区: 见下表

```字段名|类型|描述|可缺省``` :- | :- | :- |
|ret\_code|number|API 错误码|是|
|create\_on|string|API 上传时间戳|是|
|err\_msg|string|错误信息|是|


