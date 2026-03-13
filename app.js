const initialRecipes = [
  {
    id: 'r1',
    name: '番茄炒蛋',
    description: '简单又营养的家常菜，几分钟就能完成。',
    favorite: false,
  },
  {
    id: 'r2',
    name: '葱油拌面',
    description: '拌面加上香葱和酱油，瞬间变得好吃到停不下。',
    favorite: false,
  },
  {
    id: 'r3',
    name: '红烧茄子',
    description: '吸饱了酱汁的茄子，软糯入味，非常下饭。',
    favorite: false,
  },
];

const recipesKey = 'recipeListApp.recipes';

const state = {
  recipes: [],
};

function makeId() {
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  return `r_${Date.now()}_${Math.floor(Math.random() * 10000)}`;
}

function saveRecipes() {
  localStorage.setItem(recipesKey, JSON.stringify(state.recipes));
}

function normalizeRecipe(recipe) {
  return {
    id: recipe.id || makeId(),
    name: recipe.name || '',
    description: recipe.description || '',
    favorite: typeof recipe.favorite === 'boolean' ? recipe.favorite : false,
  };
}

function loadRecipes() {
  const stored = localStorage.getItem(recipesKey);
  if (stored) {
    try {
      const loaded = JSON.parse(stored);
      if (Array.isArray(loaded)) {
        state.recipes = loaded.map(normalizeRecipe);
        return;
      }
    } catch {
      // fall through
    }
  }

  state.recipes = initialRecipes.map(normalizeRecipe);
}

function toggleFavorite(recipeId) {
  const idx = state.recipes.findIndex(r => r.id === recipeId);
  if (idx === -1) return;
  state.recipes[idx].favorite = !state.recipes[idx].favorite;
  saveRecipes();
  renderRecipes();
}

function removeRecipe(recipeId) {
  state.recipes = state.recipes.filter(r => r.id !== recipeId);
  saveRecipes();
  renderRecipes();
}

function createRecipeCard(recipe) {
  const card = document.createElement('div');
  card.className = 'recipe';

  const content = document.createElement('div');
  const titleRow = document.createElement('div');
  titleRow.style.display = 'flex';
  titleRow.style.alignItems = 'center';
  titleRow.style.justifyContent = 'space-between';

  const title = document.createElement('h3');
  title.textContent = recipe.name;
  titleRow.appendChild(title);

  const starBtn = document.createElement('button');
  starBtn.className = 'favorite';
  starBtn.type = 'button';
  starBtn.title = recipe.favorite ? '取消收藏' : '添加收藏';
  starBtn.textContent = recipe.favorite ? '★' : '☆';
  starBtn.addEventListener('click', () => toggleFavorite(recipe.id));

  titleRow.appendChild(starBtn);

  const detail = document.createElement('p');
  detail.textContent = recipe.description;

  content.append(titleRow, detail);

  const deleteBtn = document.createElement('button');
  deleteBtn.className = 'delete';
  deleteBtn.type = 'button';
  deleteBtn.textContent = '删除';
  deleteBtn.addEventListener('click', () => removeRecipe(recipe.id));

  card.append(content, deleteBtn);
  return card;
}

function renderRecipes() {
  const container = document.getElementById('recipes');
  container.innerHTML = '';

  if (state.recipes.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'empty';
    empty.textContent = '当前还没有食谱，快添加一个吧！';
    container.appendChild(empty);
    return;
  }

  const sorted = [...state.recipes].sort((a, b) => {
    if (a.favorite === b.favorite) return 0;
    return a.favorite ? -1 : 1;
  });

  sorted.forEach(recipe => {
    const card = createRecipeCard(recipe);
    container.appendChild(card);
  });
}

function addRecipe(event) {
  event.preventDefault();

  const nameInput = document.getElementById('recipeName');
  const descInput = document.getElementById('recipeDesc');

  const name = nameInput.value.trim();
  const description = descInput.value.trim();

  if (!name) {
    nameInput.focus();
    return;
  }

  state.recipes.unshift({
    id: makeId(),
    name,
    description,
    favorite: false,
  });

  saveRecipes();
  renderRecipes();

  nameInput.value = '';
  descInput.value = '';
  nameInput.focus();
}

function setup() {
  loadRecipes();
  renderRecipes();

  const form = document.getElementById('recipeForm');
  form.addEventListener('submit', addRecipe);
}

window.addEventListener('DOMContentLoaded', setup);
