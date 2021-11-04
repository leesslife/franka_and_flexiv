var mqtt=require('mqtt')
var client=mqtt.connect("mqtt://localhost:18080");

//连接后不断发布temp topic主题

client.on("connect",(e)=>{
    console.log("success connect mqtt server");
    setInterval(()=>{
        client.publish("temp","25.6");
        console.log("send it");
    },1000);
});

