const search = document.getElementById('search');
const index = document.getElementById('index');
if (search && index) {
  const filter = () => {
    const query = search.value.trim().toLowerCase();
    index.open = Boolean(query);
    for (const row of document.querySelectorAll('#classes li')) {
      row.hidden = !row.textContent.toLowerCase().includes(query);
    }
  };
  search.addEventListener('input', filter);
  search.value = new URLSearchParams(window.location.search).get('q') || '';
  filter();
}
