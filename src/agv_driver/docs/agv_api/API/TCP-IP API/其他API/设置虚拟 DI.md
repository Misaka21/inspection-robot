![...](..\..\TCP-IP%20API\其他API\images\设置虚拟%20DI.001.png)

**设置虚拟 DI**


```<p>注意：</p><p>该 API 用于设置机器人控制器上的虚拟 DI (Virtual Digital Iutput) 状态。</p>```

**请求**

- 编号: 6020 (0x1784)
- 名称: robot\_other\_setvdi\_req
- 描述: 设置虚拟 DI
- JSON 数据区: 见下表

```**字段名**|**类型**|**描述**|**可缺省**``` :- | :- | :- |
|<p>id</p><p></p>|number|虚拟 DI 的 id (DI 列表中第一个虚拟 DI 的索引(index) 为 0, id 范围 [0, 7])|否|
|status|boolean|true 为高电平, false 为低电平|否|

**请求示例**


```JSON
5A 01 00 01 00 00 00 17 17 84 00 00 00 00 00 00
{"id":1,"status":false}
// 序列化后十六进制为：
7B 22 69 64 22 3A 31 2C 22 73 74 61 74 75 73 22 
3A 66 61 6C 73 65 7D```

**响应**

- 编号: 16020 (0x3E94)
- 名称: robot\_other\_setvdi\_res
- 描述: 设置虚拟 DI 的响应
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
5A 01 00 01 00 00 00 0E 3E 94 17 84 00 00 00 00
7B 22 72 65 74 5F 63 6F 64 65 22 3A 30 7D```


