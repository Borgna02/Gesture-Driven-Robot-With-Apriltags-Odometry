//  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //

var notifier = null;

function buildWs(url, onmessage)
{
    notifier = new WebSocket(url);

    notifier.onopen = function()
    {
        console.log(url, "Connected");
    };

    notifier.onclose = function()
    {
        notifier = null;
        console.log(url, "Disconnected");
        alert("Frontend is disconnected.")
    }

    notifier.onmessage = onmessage;
}

(function ($) {
    $.fn.serializeFormJSON = function () {

        var o = {};
        var a = this.serializeArray();

        $.each(a, function () {
            if (o[this.name]) {
                if (!o[this.name].push) {
                    o[this.name] = [o[this.name]];
                }
                o[this.name].push(this.value || '');
            } else {
                o[this.name] = this.value || '';
            }
        });
        return o;
    };
})(jQuery);

//  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //

$(document).ready(function()
{
    if ("WebSocket" in window)
    {
        var proto = "";

        if (document.location.protocol === 'https:')
            proto = "wss://";

        else
            proto = "ws://"

        buildWs(proto + location.host + "/frontend-controller",
                function(evt)
                {
                    if (typeof evt.data === 'string')
                    {
                        var m = JSON.parse(evt.data);
                        var cmd = m["command"];

                        //console.log(m["code"]);

                        if (cmd === "EVAL")
                            eval?.(m["code"]);

                        else if (cmd === "SET_BODY")
                            document.querySelector('mainSection').innerHTML = m["code"];
                    }

                    else
                    {}
                });
    }

    else
        alert('WebSocket NOT supported by your Browser!');

});

//  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //  //

