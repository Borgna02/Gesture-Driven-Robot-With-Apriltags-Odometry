$('#$$MODAL_ID$$').on('shown.bs.modal',
                      function()
                      {
                          $#ON_SHOWN#$
                      });

$('#$$MODAL_ID$$').on('hidden.bs.modal',
                      function()
                      {
                          $#ON_HIDDEN#$
                          $('#$$MODAL_ID$$').remove();
                      });

$('#$$MODAL_ID$$').modal('show');
