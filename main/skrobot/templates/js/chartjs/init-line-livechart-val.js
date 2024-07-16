var $$ID$$_samples_buffer = [];

for(var i=0; i < $$PARENT_ID$$_sampleslimit; i++)
    $$ID$$_samples_buffer.push(0);

var $$ID$$_dataset = {
    label: '$$LABEL$$',
    tension: 0.3,
    data: $$ID$$_samples_buffer,
    borderColor: '$$COLOR$$',
    borderWidth: 1,
    animation: $$ANIMATION$$,
    fill: false,
    LAST_VALUE : 0
};

$$PARENT_ID$$_chart.data.datasets.push($$ID$$_dataset);

$%$

{
    "defaultsForVariables" : {
        "COLOR" : "green",
        "ANIMATION" : "false"
    }
}
