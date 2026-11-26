// Converte o JSON bruto da Overpass (gerado por osm_download) no arquivo
// map.cpp, que a aplicacao inclui com #include "map.cpp".
//
// Compilar:  c++ -std=c++17 -O2 -Wall -Wextra -Isrc tools/osm2map.cpp -o osm2map
// Usar:      ./osm2map data/raw.json data/map.cpp [municipio ...]
//            (os nomes dos municipios so entram no comentario de cabecalho;
//             se existir data/raw.json.query.txt, a consulta tambem entra)
//
// Grafo completo: todo nodo OSM de uma via mantida e um nodo do grafo, e cada
// par de nodos consecutivos da via vira uma ou duas arestas.
//
// Pipeline:
//   1. le nodos e vias do JSON
//   2. filtra vias dirigiveis (tipo de via, acesso, servicos de estacionamento)
//   3. cada par de nodos consecutivos de uma via vira uma aresta (duas, se a
//      via for de mao dupla), com a distancia Haversine entre eles
//   4. mantem a maior componente FORTEMENTE conexa (garante rota entre
//      qualquer par de nodos, respeitando as maos unicas)
//   5. grava o map.cpp de forma deterministica (ordenado)
//
// Dados do mapa: (c) OpenStreetMap contributors, licenca ODbL
// (https://www.openstreetmap.org/copyright).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "format.hpp"
#include "mini_json.hpp"

namespace {

// ---------------------------------------------------------------- geometria

constexpr double kEarthRadiusM = 6371008.8;
constexpr double kPi = 3.14159265358979323846;

struct LatLon {
  double lat, lon;
};

double Haversine(const LatLon& a, const LatLon& b) {
  const double rad = kPi / 180.0;
  double dlat = (b.lat - a.lat) * rad;
  double dlon = (b.lon - a.lon) * rad;
  double la = a.lat * rad, lb = b.lat * rad;
  double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
             std::cos(la) * std::cos(lb) * std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

// ------------------------------------------------------------------ filtros

// Tipos de via mantidos (dirigiveis). Excluidos: footway, path, steps,
// cycleway, track, pedestrian, etc.
const std::set<std::string> kDrivable = {
    "motorway",  "motorway_link", "trunk",       "trunk_link",
    "primary",   "primary_link",  "secondary",   "secondary_link",
    "tertiary",  "tertiary_link", "unclassified", "residential",
    "living_street", "service",   "road"};

// service=* descartados por padrao (poluem o grafo com becos de garagem).
const std::set<std::string> kSkippedService = {"parking_aisle", "driveway",
                                               "drive-through"};

std::string Tag(const minijson::Value* tags, const char* key) {
  if (!tags) return "";
  const minijson::Value* v = tags->Get(key);
  return (v && v->type == minijson::Value::Type::String) ? v->string : "";
}

bool Blocked(const std::string& v) { return v == "no" || v == "private"; }

struct WayInfo {
  long long id = 0;
  std::vector<long long> nodes;
  std::string name;     // name, senao ref, senao ""
  std::string highway;
  int oneway = 0;       // 0 = mao dupla, 1 = no sentido da way, -1 = sentido inverso
};

// Estatisticas do filtro, para o relatorio final.
struct FilterStats {
  int not_drivable = 0, blocked_access = 0, skipped_service = 0, too_short = 0;
  int oneway_unknown_value = 0;
};

bool ClassifyWay(const minijson::Value& el, bool keep_all_service, WayInfo& w,
                 FilterStats& st) {
  const minijson::Value* tags = el.Get("tags");
  std::string hw = Tag(tags, "highway");
  if (!kDrivable.count(hw)) {
    ++st.not_drivable;
    return false;
  }
  // Acesso: motor_vehicle > vehicle > access (o mais especifico vence).
  std::string acc = Tag(tags, "motor_vehicle");
  if (acc.empty()) acc = Tag(tags, "vehicle");
  if (acc.empty()) acc = Tag(tags, "access");
  if (Blocked(acc)) {
    ++st.blocked_access;
    return false;
  }
  if (hw == "service" && !keep_all_service && kSkippedService.count(Tag(tags, "service"))) {
    ++st.skipped_service;
    return false;
  }

  w.highway = hw;
  w.name = Tag(tags, "name");
  if (w.name.empty()) w.name = Tag(tags, "ref");

  // oneway: tag explicita vence; senao rotatoria/autoestrada sao implicitas.
  std::string ow = Tag(tags, "oneway");
  std::string junction = Tag(tags, "junction");
  if (ow == "yes" || ow == "true" || ow == "1") {
    w.oneway = 1;
  } else if (ow == "-1" || ow == "reverse") {
    w.oneway = -1;
  } else if (ow == "no" || ow == "false" || ow == "0") {
    w.oneway = 0;
  } else {
    if (!ow.empty()) ++st.oneway_unknown_value;  // ex.: reversible, alternating -> mao dupla
    bool implied = (junction == "roundabout" || junction == "circular" ||
                    hw == "motorway" || hw == "motorway_link");
    w.oneway = (ow.empty() && implied) ? 1 : 0;
  }

  const minijson::Value* nd = el.Get("nodes");
  if (!nd || nd->type != minijson::Value::Type::Array) return false;
  for (const auto& n : nd->array) w.nodes.push_back(n.integer);
  if (w.nodes.size() < 2) {
    ++st.too_short;
    return false;
  }
  w.id = el.Get("id") ? el.Get("id")->integer : 0;
  return true;
}

// ------------------------------------------------------------- grafo bruto

struct RawEdge {
  long long from, to, way_id;
  double distance;
  std::string street, highway;
};

// Maior componente fortemente conexa (Kosaraju iterativo, sem recursao para
// nao estourar a pilha em mapas grandes). Devolve, para cada vertice, se esta
// na maior componente.
std::vector<char> LargestScc(int n, const std::vector<std::pair<int, int>>& arcs,
                             int& comp_count, int& largest_size) {
  std::vector<std::vector<int>> adj(n), radj(n);
  for (const auto& a : arcs) {
    adj[a.first].push_back(a.second);
    radj[a.second].push_back(a.first);
  }
  std::vector<int> order;
  order.reserve(n);
  std::vector<char> seen(n, 0);
  for (int s = 0; s < n; ++s) {
    if (seen[s]) continue;
    std::vector<std::pair<int, size_t>> stack;
    stack.push_back({s, 0});
    seen[s] = 1;
    while (!stack.empty()) {
      auto& [v, i] = stack.back();
      if (i < adj[v].size()) {
        int w = adj[v][i++];
        if (!seen[w]) {
          seen[w] = 1;
          stack.push_back({w, 0});
        }
      } else {
        order.push_back(v);
        stack.pop_back();
      }
    }
  }
  std::vector<int> comp(n, -1);
  std::vector<int> sizes;
  for (int k = n - 1; k >= 0; --k) {
    int s = order[k];
    if (comp[s] != -1) continue;
    int id = static_cast<int>(sizes.size());
    sizes.push_back(0);
    std::vector<int> stack{s};
    comp[s] = id;
    while (!stack.empty()) {
      int v = stack.back();
      stack.pop_back();
      ++sizes[id];
      for (int w : radj[v]) {
        if (comp[w] == -1) {
          comp[w] = id;
          stack.push_back(w);
        }
      }
    }
  }
  comp_count = static_cast<int>(sizes.size());
  int best = 0;
  for (int i = 1; i < comp_count; ++i)
    if (sizes[i] > sizes[best]) best = i;
  largest_size = comp_count ? sizes[best] : 0;
  std::vector<char> in(n, 0);
  for (int v = 0; v < n; ++v) in[v] = (comp[v] == best);
  return in;
}

// ------------------------------------------------------------------- saida

// Escapa uma string para literal C++. UTF-8 passa direto (o arquivo e UTF-8).
std::string CppEscape(const std::string& s) {
  std::string out;
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      case '?': out += "\\?"; break;  // evita trigrafos
      default:
        if (c < 0x20 || c == 0x7f) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\%03o", c);  // 3 digitos: nao "gruda" no proximo
          out += buf;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  return out;
}

std::string Fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, f, v);
  return buf;
}

// Escreve um texto multilinha como comentarios // (cuidado com '\' no fim da
// linha, que continuaria o comentario).
void WriteComment(std::ostream& out, const std::string& text, const char* prefix) {
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    while (!line.empty() && (line.back() == '\\' || line.back() == '\r')) line.pop_back();
    out << prefix << line << "\n";
  }
}

std::string ReadFile(const std::string& path, bool& ok) {
  std::ifstream in(path, std::ios::binary);
  ok = static_cast<bool>(in);
  if (!ok) return "";
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

// --------------------------------------------------------------------- main

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Uso: " << argv[0] << " <raw.json> <map.cpp> [municipio ...]\n";
    return 1;
  }
  const std::string in_path = argv[1], out_path = argv[2];
  std::string cities;
  for (int i = 3; i < argc; ++i) cities += (i > 3 ? ", " : "") + std::string(argv[i]);

  bool ok = false;
  std::string text = ReadFile(in_path, ok);
  if (!ok) {
    std::cerr << "Nao foi possivel ler " << in_path << "\n";
    return 1;
  }

  // ---- 1. leitura -----------------------------------------------------
  std::unordered_map<long long, LatLon> osm_nodes;
  std::vector<WayInfo> ways;
  FilterStats fs;
  int ways_total = 0;
  minijson::Value meta;
  try {
    minijson::ReadOverpass(text, meta, [&](const minijson::Value& el) {
      const minijson::Value* type = el.Get("type");
      if (!type) return;
      if (type->string == "node") {
        const minijson::Value *id = el.Get("id"), *lat = el.Get("lat"), *lon = el.Get("lon");
        if (id && lat && lon) osm_nodes[id->integer] = LatLon{lat->number, lon->number};
      } else if (type->string == "way") {
        ++ways_total;
        WayInfo w;
        if (ClassifyWay(el, false, w, fs)) ways.push_back(std::move(w));
      }
    });
  } catch (const std::exception& e) {
    std::cerr << "Erro ao ler " << in_path << ": " << e.what() << "\n";
    return 1;
  }
  text.clear();
  text.shrink_to_fit();
  if (ways.empty()) {
    std::cerr << "Nenhuma via dirigivel encontrada em " << in_path
              << " (o nome do municipio existe no OSM com admin_level=8?)\n";
    return 2;
  }

  // ---- 2/3. arestas entre nodos consecutivos --------------------------
  std::vector<RawEdge> edges;
  int ways_missing_nodes = 0, kept_ways = 0, repeated_nodes = 0;
  for (const auto& w : ways) {
    bool complete = true;
    for (long long id : w.nodes)
      if (!osm_nodes.count(id)) { complete = false; break; }
    if (!complete) { ++ways_missing_nodes; continue; }
    ++kept_ways;
    for (size_t i = 1; i < w.nodes.size(); ++i) {
      long long a = w.nodes[i - 1], b = w.nodes[i];
      if (a == b) { ++repeated_nodes; continue; }  // nodo repetido na via: sem aresta
      RawEdge e;
      e.way_id = w.id;
      e.street = w.name;
      e.highway = w.highway;
      e.distance = Haversine(osm_nodes[a], osm_nodes[b]);
      if (w.oneway >= 0) {  // sentido da via (ou mao dupla)
        RawEdge f = e;
        f.from = a; f.to = b;
        edges.push_back(std::move(f));
      }
      if (w.oneway <= 0) {  // sentido inverso (mao dupla, ou oneway=-1)
        RawEdge r = e;
        r.from = b; r.to = a;
        edges.push_back(std::move(r));
      }
    }
  }
  const size_t edges_before = edges.size();

  // ---- 4. maior componente fortemente conexa --------------------------
  std::unordered_map<long long, int> dense;
  std::vector<long long> dense_ids;
  for (const auto& e : edges)
    for (long long id : {e.from, e.to})
      if (dense.emplace(id, static_cast<int>(dense_ids.size())).second) dense_ids.push_back(id);
  const size_t nodes_before = dense_ids.size();
  std::vector<std::pair<int, int>> arcs;
  arcs.reserve(edges.size());
  for (const auto& e : edges) arcs.push_back({dense[e.from], dense[e.to]});
  int comp_count = 0, largest = 0;
  std::vector<char> keep = LargestScc(static_cast<int>(dense_ids.size()), arcs, comp_count, largest);
  if (largest < 2) {
    std::cerr << "Maior componente conexa tem " << largest << " nodo(s); nada a gerar.\n";
    return 2;
  }
  {
    std::vector<RawEdge> kept;
    for (auto& e : edges)
      if (keep[dense[e.from]] && keep[dense[e.to]]) kept.push_back(std::move(e));
    edges.swap(kept);
  }

  // ---- ordenacao deterministica ---------------------------------------
  std::stable_sort(edges.begin(), edges.end(), [](const RawEdge& a, const RawEdge& b) {
    if (a.from != b.from) return a.from < b.from;
    if (a.to != b.to) return a.to < b.to;
    if (a.way_id != b.way_id) return a.way_id < b.way_id;
    return a.distance < b.distance;
  });
  std::vector<long long> node_ids;
  for (size_t v = 0; v < dense_ids.size(); ++v)
    if (keep[v]) node_ids.push_back(dense_ids[v]);
  std::sort(node_ids.begin(), node_ids.end());

  // ---- estatisticas / validacao ---------------------------------------
  size_t unnamed = 0, zero_len = 0;
  double total_m = 0, longest = 0;
  for (const auto& e : edges) {
    unnamed += e.street.empty();
    zero_len += (e.distance < 0.01);
    total_m += e.distance;
    longest = std::max(longest, e.distance);
  }
  double min_lat = 90, max_lat = -90, min_lon = 180, max_lon = -180;
  for (long long id : node_ids) {
    const LatLon& p = osm_nodes[id];
    min_lat = std::min(min_lat, p.lat); max_lat = std::max(max_lat, p.lat);
    min_lon = std::min(min_lon, p.lon); max_lon = std::max(max_lon, p.lon);
  }

  // ---- 5. escrita ------------------------------------------------------
  std::ofstream out(out_path, std::ios::binary);
  if (!out) {
    std::cerr << "Nao foi possivel criar " << out_path << "\n";
    return 1;
  }
  std::string base_ts, copyright;
  if (const auto* osm3s = meta.Get("osm3s")) {
    base_ts = Tag(osm3s, "timestamp_osm_base");
    copyright = Tag(osm3s, "copyright");
  }
  out << "// map.cpp -- GERADO por osm2map. NAO EDITE; regenere com `make map`.\n"
      << "//\n"
      << "// Contains data (c) OpenStreetMap contributors, licensed under the\n"
      << "// Open Database License (ODbL): https://www.openstreetmap.org/copyright\n";
  if (!copyright.empty()) WriteComment(out, "Aviso da Overpass: " + copyright, "// ");
  out << "//\n";
  if (!cities.empty()) out << "// Municipios:        " << cities << "\n";
  if (!base_ts.empty()) out << "// Base OSM em:       " << base_ts << "\n";
  // So o nome do arquivo (nao o caminho): o map.cpp e versionado e nao deve
  // carregar caminhos da maquina de quem o gerou.
  const size_t slash = in_path.find_last_of("/\\");
  out << "// Arquivo de origem: " << (slash == std::string::npos ? in_path : in_path.substr(slash + 1)) << "\n"
      << "// Conteudo:          " << node_ids.size() << " nodos, " << edges.size()
      << " arestas (unidirecionais)\n"
      << "// Formato:           ver format.hpp (Node, Edge)\n";
  bool qok = false;
  std::string query = ReadFile(in_path + ".query.txt", qok);
  if (qok) {
    out << "//\n// Consulta Overpass usada:\n";
    WriteComment(out, query, "//   ");
  }
  out << "\n#include \"format.hpp\"\n\n";

  out << "static const int NodeCount = " << node_ids.size() << ";\n"
      << "static const int EdgeCount = " << edges.size() << ";\n\n"
      << "// Caixa delimitadora (graus), util para o zoom inicial.\n"
      << "static const double BBoxMinLat = " << Fmt("%.7f", min_lat) << ";\n"
      << "static const double BBoxMaxLat = " << Fmt("%.7f", max_lat) << ";\n"
      << "static const double BBoxMinLon = " << Fmt("%.7f", min_lon) << ";\n"
      << "static const double BBoxMaxLon = " << Fmt("%.7f", max_lon) << ";\n\n";

  out << "static const Node Nodes[] = {\n";
  for (long long id : node_ids) {
    const LatLon& p = osm_nodes[id];
    out << "  {" << id << ", " << Fmt("%.7f", p.lat) << ", " << Fmt("%.7f", p.lon) << "},\n";
  }
  out << "};\n\n";

  out << "// from, to, distance(m), street, way_id, highway\n"
      << "static const Edge Edges[] = {\n";
  for (const auto& e : edges) {
    out << "  {" << e.from << ", " << e.to << ", " << Fmt("%.2f", e.distance) << ", \""
        << CppEscape(e.street) << "\", " << e.way_id << ", \"" << CppEscape(e.highway)
        << "\"},\n";
  }
  out << "};\n\n";

  out << "static_assert(sizeof(Nodes) / sizeof(Nodes[0]) == NodeCount, \"NodeCount\");\n"
      << "static_assert(sizeof(Edges) / sizeof(Edges[0]) == EdgeCount, \"EdgeCount\");\n";
  out.close();
  if (!out) {
    std::cerr << "Erro ao gravar " << out_path << "\n";
    return 1;
  }

  // ---- relatorio -------------------------------------------------------
  std::cerr << "== osm2map (grafo completo) ==\n"
            << "  nodos OSM lidos:            " << osm_nodes.size() << "\n"
            << "  vias lidas / mantidas:      " << ways_total << " / " << kept_ways << "\n"
            << "    descartadas: nao dirigivel " << fs.not_drivable << ", acesso " << fs.blocked_access
            << ", servico " << fs.skipped_service << ", curtas " << fs.too_short
            << ", nodos ausentes " << ways_missing_nodes << "\n"
            << "  nodos repetidos em sequencia (ignorados): " << repeated_nodes << "\n"
            << "  antes da componente conexa: " << nodes_before << " nodos, " << edges_before << " arestas\n"
            << "  componentes fortemente conexas: " << comp_count << " (maior: " << largest << " nodos)\n"
            << "  FINAL: " << node_ids.size() << " nodos, " << edges.size() << " arestas\n"
            << "  extensao total: " << Fmt("%.1f", total_m / 1000.0) << " km (soma das arestas), "
            << "maior aresta: " << Fmt("%.0f", longest) << " m\n"
            << "  arestas sem nome: " << unnamed << " (" << Fmt("%.1f", 100.0 * unnamed / edges.size())
            << "%), com distancia ~0: " << zero_len << "\n";
  if (fs.oneway_unknown_value)
    std::cerr << "  AVISO: " << fs.oneway_unknown_value << " vias com oneway de valor desconhecido (tratadas como mao dupla)\n";
  std::cerr << "Gravado em " << out_path << "\n";
  return 0;
}
