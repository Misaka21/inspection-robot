![...](..\..\TCP-IP%20API\机器人配置API\images\清除第三方%20Warning.001.png)

**清除第三方 Warning**


```<p>注意：</p><p>用于清除 54900 的 Warning 报警码。</p>```

**请求**

- 编号: 4803 (0x12C3)
- 名称: robot\_config\_clearwarning\_req
- 描述: 清除第三方 Warning
- JSON 数据区: 无

**请求示例**


```Plaintext
5A 01 00 01 00 00 00 00 12 C3 00 00 00 00 00 00```

**响应**

- 编号: 14803 (0x39D3)
- 名称: robot\_config\_clearwarning\_res
- 描述: 清除第三方 Warning 的响应
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
5A 01 00 01 00 00 00 0E 39 D3 12 C3 00 00 00 00
7B 22 72 65 74 5F 63 6F 64 65 22 3A 30 7D```


