// osm_download.cpp
// Baixa os dados brutos de vias (highway) do OpenStreetMap via Overpass API
// e salva o JSON original em disco
//
// Compilar (macOS ou Linux, requer libcurl):
//   c++ -std=c++17 -O2 osm_download.cpp -lcurl -o osm_download
//
// Usar:
//   ./osm_download raw.json Gramado Canela
//   ./osm_download raw_nova_petropolis.json "Nova Petrópolis"
//
// Dados © OpenStreetMap contributors, licenca ODbL
// (https://www.openstreetmap.org/copyright). O proprio JSON da Overpass


#include <curl/curl.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Servidores Overpass tentados em ordem.
static const std::vector<std::string> kEndpoints = {
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
};

// Tipos de via claramente nao dirigiveis, excluidos ja na consulta para
// reduzir o download. A filtragem fina fica para a etapa de conversao.
static const char* kExcludedHighways =
    "footway|path|steps|cycleway|bridleway|pedestrian|corridor|"
    "proposed|construction|raceway|elevator";

// Monta a consulta Overpass para um ou mais municipios (admin_level 8).
static std::string BuildQuery(const std::vector<std::string>& cities) {
  std::string areas;
  for (const auto& c : cities) {
    areas += "  area[\"name\"=\"" + c +
             "\"][\"boundary\"=\"administrative\"][\"admin_level\"=\"8\"];\n";
  }
  return "[out:json][timeout:180];\n"
         "(\n" + areas + ")->.a;\n"
         "way(area.a)[\"highway\"][\"highway\"!~\"" +
         std::string(kExcludedHighways) + "\"];\n"
         "out body;\n"
         ">;\n"
         "out skel qt;\n";
}

static size_t WriteToStream(char* data, size_t size, size_t n, void* user) {
  auto* out = static_cast<std::ofstream*>(user);
  out->write(data, static_cast<std::streamsize>(size * n));
  return out->good() ? size * n : 0;
}

// Faz um POST da consulta e grava a resposta em `path`.
// Retorna true se o HTTP foi 200.
static bool TryDownload(const std::string& endpoint, const std::string& query,
                        const std::string& path) {
  CURL* curl = curl_easy_init();
  if (!curl) return false;

  char* escaped = curl_easy_escape(curl, query.c_str(), 0);
  std::string body = std::string("data=") + (escaped ? escaped : "");
  curl_free(escaped);

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::cerr << "Nao foi possivel criar " << path << "\n";
    curl_easy_cleanup(curl);
    return false;
  }

  curl_easy_setopt(curl, CURLOPT_URL, endpoint.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "StreetMap/0.1");
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteToStream);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 20L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 300L);
  curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");  // aceita gzip/deflate
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

  CURLcode rc = curl_easy_perform(curl);
  long http = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http);
  curl_easy_cleanup(curl);
  out.close();

  if (rc != CURLE_OK) {
    std::cerr << "  falha de rede: " << curl_easy_strerror(rc) << "\n";
    return false;
  }
  if (http != 200) {
    std::cerr << "  HTTP " << http << "\n";
    return false;
  }
  return true;
}

// A Overpass as vezes responde 200 com um erro dentro do JSON
// (ex.: "runtime error: timeout"). Confere o basico antes de aceitar.
static bool LooksValid(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(in)),
                      std::istreambuf_iterator<char>());
  if (content.empty() || content[0] != '{') return false;
  if (content.find("\"elements\"") == std::string::npos) return false;
  if (content.find("runtime error") != std::string::npos) return false;
  return true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Uso: " << argv[0] << " <saida.json> <municipio> [municipio...]\n"
              << "Ex.: " << argv[0] << " raw.json Gramado Canela\n";
    return 1;
  }
  const std::string output = argv[1];
  std::vector<std::string> cities(argv + 2, argv + argc);

  for (const auto& c : cities) {
    if (c.find('"') != std::string::npos || c.find('\\') != std::string::npos) {
      std::cerr << "Nome de municipio invalido: " << c << "\n";
      return 1;
    }
  }

  const std::string query = BuildQuery(cities);
  std::cout << "Consulta Overpass:\n" << query << "\n";

  // Guarda a consulta ao lado do JSON (util para o cabecalho de proveniencia).
  {
    std::ofstream q(output + ".query.txt");
    q << query;
  }

  // Baixa para um arquivo temporario; so substitui o final se estiver valido.
  const std::string tmp = output + ".part";
  curl_global_init(CURL_GLOBAL_DEFAULT);

  bool ok = false;
  for (const auto& ep : kEndpoints) {
    std::cout << "Baixando de " << ep << " ...\n";
    if (TryDownload(ep, query, tmp) && LooksValid(tmp)) {
      ok = true;
      break;
    }
    std::cerr << "  resposta invalida, tentando o proximo servidor\n";
  }
  curl_global_cleanup();

  if (!ok) {
    std::remove(tmp.c_str());
    std::cerr << "Falha em todos os servidores. Tente novamente em alguns minutos.\n";
    return 2;
  }

  std::remove(output.c_str());
  if (std::rename(tmp.c_str(), output.c_str()) != 0) {
    std::cerr << "Nao foi possivel renomear " << tmp << " para " << output << "\n";
    return 3;
  }

  std::ifstream sz(output, std::ios::binary | std::ios::ate);
  std::cout << "Salvo em " << output << " (" << sz.tellg() / 1024 << " KiB)\n"
            << "Dados © OpenStreetMap contributors (ODbL).\n";
  return 0;
}
