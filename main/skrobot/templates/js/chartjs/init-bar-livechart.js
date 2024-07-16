var $$ID$$_cfg = {
    type: 'bar',
    data: {
        labels: $$LABELS$$,
        datasets: []
    },
    options: {
        indexAxis: '$$INDEX_AXIS$$',
        responsive: true,
        legend: {
            display: $$LEGEND_DYSPLAY$$
        },
        elements: {
            bar: {
                borderWidth: $$BAR_BORDER$$
            }
        },
        scales: {
            x: {
                min: $$MIN$$,
                suggestedMax: $$MAX$$,
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

var $$ID$$_chart = new Chart('$$ID$$', $$ID$$_cfg);

function $$ID$$_showData()
{
    $$ID$$_chart.data.datasets.forEach((dataset) =>
                                       {
                                           dataset.data = dataset.LAST_DATA;
                                       });

    $$ID$$_chart.update();
}


$%$

{
    "defaultsForVariables" : {
        "LEGEND_DYSPLAY" : "true",
        "MAX" : "",
        "MIN" : "",
        "BAR_BORDER" : "2",
        "INDEX_AXIS" : ""
    }
}
