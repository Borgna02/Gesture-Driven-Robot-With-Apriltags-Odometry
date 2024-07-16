$$PARENT_ID$$_chart.data

var $$ID$$_dataset = {
    label: '$$LABEL$$',
    data: [],
    borderColor: '$$BORDER_COLOR$$',
    backgroundColor : '$$BG_COLOR$$',
    borderWidth: 1,
    borderRadius: 3,
    borderSkipped: false,
    animation: $$ANIMATION$$,
    LAST_DATA : []
};

$$PARENT_ID$$_chart.data.datasets.push($$ID$$_dataset);

$%$

{
    "defaultsForVariables" : {
        "BG_COLOR" : "green",
        "BORDER_COLOR" : "black",
        "ANIMATION" : "false"
    }
}
