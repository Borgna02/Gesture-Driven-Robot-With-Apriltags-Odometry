$('#$$TARGET_ID$$').load('$$URL$$',
                         function()
                         {
                             $#ON_LOAD#$
                         });

notifier.send(JSON.stringify(
                  {
                      cmd : 'URL_REQ',
                      url : '$$URL$$'
                  }));
