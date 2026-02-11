![...](..\..\TCP-IP%20API\附录\images\API%20错误码.001.png)

**API 错误码**

API 错误码为响应数据区 JSON 对象包含的错误码（ ret\_code ），用于指示对请求执行不成功或请求出错等错误。若 ret\_code 为 0 或缺省，说明没有错误，若不为0，说明发生了错误，错误码见下表：

错误码只会出现在响应数据区的  JSON  数据中

```**错误码**|**名称**|**描述**``` :- | :- |
|40000|req\_unavailable|请求不可用|
|40001|param\_missing|必要的请求参数缺失|
|40002|param\_type\_error|请求参数类型错误|
|40003|param\_illegal|请求参数不合法|
|40004|mode\_error|运行模式错误|
|40005|illegal\_map\_name|非法的地图名|
|40006|programming\_dsp|正在烧写固件|
|40007|program\_dsp\_error|烧写固件错误|
|40008|illegal\_filename|文件名非法|
|40010|shutdown\_error|关机指令出现错误|
|40011|reboot\_error|重启指令出现错误|
|40012|dispatching|调度系统控制中|
|40013|robod\_error|robod 错误|
|40014|robod\_warning|robod 警告|
|40015|manual\_charging|正在手动充电，不能运动|
|40016|emc\_status|急停状态中|
|40020|locked|控制权被抢占|
|40050|map\_parse\_error\_|地图解析出错|
|40051|map\_not\_exists|地图不存在|
|40052|load\_map\_error|加载地图错误|
|40053|load\_mapobj\_error|重载地图错误|
|40054|empty\_map|空地图|
|40055|file\_not\_exists|文件不存在|
|40056|map\_convert\_error|地图转换失败|
|40057|rawmap\_not\_exists|当前无可用 rawmap 文件|
|40058|calib\_file\_not\_exists|当前无可用 calib 文件|
|40060|audio\_not\_exists|音频文件不存在|
|40061|audio\_play\_error|播放音频出错|
|40062|upload\_audio\_error|上传音频文件失败|
|40063|audio\_is\_playing|音频正在播放中|
|40069|model\_save\_error|保存模型文件出错|
|40070|model\_parse\_error|模型文件解析错误|
|40071|calibration\_parse\_error|标定数据解析错误|
|40072|calibration\_save\_error|保存标定文件出错|
|40073|calibration\_clear\_error|清除标定数据出错|
|40100|req\_timeout|请求执行超时|
|40101|req\_forbidden|请求被禁止|
|40102|robot\_busy|机器人繁忙|
|40199|robot\_internal\_error|内部错误|
|40200|tasklist\_parse\_error|解析任务链错误|
|40201|illegal\_tasklist\_name|任务链名字非法|
|40202|tasklist\_not\_exists|任务链不存在|
|40203|tasklist\_executing|任务链正在执行中|
|40300|set\_param\_type\_error|设置参数类型错误|
|40301|set\_param\_not\_exists|设置的参数不存在|
|40302|set\_param\_error|设置参数出错|
|40310|save\_param\_type\_error|设置并保存参数类型错误|
|40311|save\_param\_not\_exists|设置并保存参数不存在|
|40312|save\_param\_error|设置并保存参数出错|
|40320|reload\_param\_type\_error|重载的参数类型错误|
|40321|reload\_param\_not\_exists|重载的参数不存在|
|40322|reload\_param\_error|重载参数出错|
|40400|src\_require\_error|获取控制权错误|
|40401|src\_release\_error|释放控制权错误|
|41000|init\_status\_error|初始化状态错误|
|41001|loadmap\_status\_error|地图载入状态错误。通常是未获取到里程数据或激光数据，或者里程数据或激光数据未更新。|
|41002|reloc\_status\_error|重定位状态错误|
|41003|reloc\_no\_robot\_home|找不到重定位的 robotHome|
|41004|confidence\_too\_low|置信度过低|
|41100|no\_start\_pos|找不到起点|
|41101|no\_ready\_pos|找不到准备点|
|41102|no\_end\_pod|找不到终点|
|41103|no\_charge\_pos|充电点不存在|
|41200|speed\_illegal|速度值非法|
|42000|roller\_connect\_error|辊筒或皮带连接错误|
|42001|roller\_type\_unknown|辊筒或皮带类型未知|
|42002|roller\_cmd\_unsupported|辊筒或皮带不支持该指令|
|42003|jack\_connect\_error|顶升机构连接错误|
|42004|jack\_type\_unknown|顶升机构类型未知|
|42005|jack\_cmd\_unsupported|顶升机构不支持该指令|
|43000|lift\_failed|升降操作出错|
|44000|redis\_conn\_error|redis 连接错误|
|44001|subchannel\_error|redis 订阅错误|
|60000|robot\_error\_wtype\_res|错误的报文类型，若用户将某类型的报文发错了端口将得到这个响应|
|60001|robot\_error\_utype\_res|未知的报文类型, 报文类型号未在上文中定义|
|60002|robot\_error\_data\_res|错误的数据区，当数据区无法反序列化为 JSON 对象时将得到这个响应|
|60003|robot\_error\_version\_res|协议版本错误时得到的响应|
|60004|robot\_error\_hugedata\_res|数据区过大，服务器会主动断连接，限制 200 MB|
|…|…|…|


