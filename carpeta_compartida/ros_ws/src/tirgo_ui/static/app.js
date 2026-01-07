const elOptions = document.getElementById('options');
const elMsg = document.getElementById('msg');
const elLast = document.getElementById('last-text');
const elRem = document.getElementById('remaining');
const btns = elOptions.querySelectorAll('button');

function setEnabled(on){
  if(on){
    elOptions.classList.remove('hidden');
    btns.forEach(b => b.disabled = false);
  }else{
    btns.forEach(b => b.disabled = true);
    elOptions.classList.add('hidden');
  }
}

async function poll(){
  try{
    const r = await fetch('/state', {cache:'no-store'});
    const s = await r.json();
    elLast.textContent = s.last_text ? `último: “${s.last_text}”` : '';
    elRem.textContent = s.hotword ? `(${s.remaining}s restantes)` : '';
    setEnabled(!!s.hotword);
  }catch(e){
    console.error('state error', e);
    setEnabled(false);
  }finally{
    setTimeout(poll, 500);
  }
}

async function press(action){
  try{
    const r = await fetch(`/press/${action}`, {method:'POST'});
    const j = await r.json();
    elMsg.textContent = j.ok ? `✅ enviado: ${j.pressed}` : `❌ ${j.error||'error'}`;
  }catch(e){
    elMsg.textContent = '❌ error';
  }finally{
    setTimeout(()=> elMsg.textContent = '', 1500);
  }
}

elOptions.addEventListener('click', (e)=>{
  if(e.target.tagName === 'BUTTON'){ press(e.target.getAttribute('data-action')); }
});

setEnabled(false);
poll();
