// Greedy Nearest Neighbor O(n²)
// Menentukan urutan rumah yang akan dikunjungi
// 1. Mulai dari gudang (index 0)
// 2. Cari rumah terdekat
// 3. Kunjungi rumah tersebut
// 4. Tandai sebagai visited
// 5. Ulangi hingga semua rumah selesai
// 6. Kembali ke gudang

function dist(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function greedyNearestNeighbor(nodes) {
  const n = nodes.length;
  const visited = new Array(n).fill(false);
  visited[0] = true;
  const route = [0];
  let current = 0;

  for (let step = 0; step < n - 1; step++) {
    let nearest = -1;
    let nearestD = Infinity;
    for (let j = 1; j < n; j++) {
      if (!visited[j]) {
        const d = dist(nodes[current], nodes[j]);
        if (d < nearestD) {
          nearestD = d;
          nearest = j;
        }
      }
    }
    if (nearest !== -1) {
      visited[nearest] = true;
      route.push(nearest);
      current = nearest;
    }
  }
  route.push(0);
  return route;
}
