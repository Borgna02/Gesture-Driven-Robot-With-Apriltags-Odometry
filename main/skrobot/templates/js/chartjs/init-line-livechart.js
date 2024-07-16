var $$ID$$_sampleslimit = $$SAMPLES_LIMIT$$;
var $$ID$$_xValues = [];

var $$ID$$_cfg = {
    type: 'line',
    data: {
        labels: $$ID$$_xValues,
        datasets: []
    },
    options: {
        legend: {
            display: $$LEGEND_DYSPLAY$$
        },
        elements: {
            point:{
                radius: $$POINT_RADIUS$$
            }
        },
        scales: {
            y: {
                beginAtZero: true
            },
            x: {

            }
        },
        plugins : {
            title: {
                display: true,
                text: '$$LABEL$$'
            },
            subtitle: {
                display: true,
                text: '[$$UDM$$]'
            }
        },
        maintainAspectRatio: false
    }
  };

var $$ID$$_chart = new Chart(document.getElementById('$$ID$$').getContext('2d'), $$ID$$_cfg);


for(var i=0; i<$$ID$$_sampleslimit; i++)
    $$ID$$_xValues.push(0);

function $$ID$$_addData()
{
    var now = new Date();
    var timeString = now.getHours()+':'+now.getMinutes()/*+'.'+now.getSeconds()*/;
    $$ID$$_chart.data.labels.push(timeString);

    $$ID$$_chart.data.datasets.forEach((dataset) =>
                                       {
                                           dataset.data.push(dataset.LAST_VALUE);
                                       });
    $$ID$$_chart.update();

    if ($$ID$$_xValues.length < $$ID$$_sampleslimit)
        return;

    $$ID$$_chart.data.labels.shift();
    $$ID$$_chart.data.datasets.forEach((dataset) =>
                                       {
                                           dataset.data.shift();
                                       });
}

$%$

{
    "defaultsForVariables" : {
        "SAMPLES_LIMIT" : "20",
        "LEGEND_DYSPLAY" : "true",
        "POINT_RADIUS" : "0"
    }
}
