$("#$$FORM_ID$$").on("submit",
                     function(e)
                     {
                         e.preventDefault();

                         var data = $(this).serializeFormJSON();
                         data['cmd'] = 'FORM';
                         var s = JSON.stringify(data);
                         notifier.send(s);

                         //console.log(s.length, s);
                     });
