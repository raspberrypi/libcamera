#pragma once

#include <string>
#include <vector>

#include "types.h"

namespace tiling {

class Stage;
class InputStage;
class OutputStage;

class Pipeline
{
public:
	struct Config {
		Config(Length2 const &_max_tile_size, Length2 const &_min_tile_size)
			: max_tile_size(_max_tile_size), min_tile_size(_min_tile_size) {}
		Length2 max_tile_size;
		Length2 min_tile_size;
	};

	Pipeline(char const *name, Config const &config);
	Config const &GetConfig() const;
	void AddStage(Stage *stage);
	void AddInputStage(InputStage *input_stage);
	void AddOutputStage(OutputStage *output_stage);
	void Tile(void *mem, size_t num_items, size_t item_size, Length2 *grid);

private:
	int tileDirection(Dir dir, void *mem, size_t num_items, size_t item_size);
	void reset();
	std::string name_;
	Config config_;
	std::vector<Stage *> stages_;
	std::vector<InputStage *> inputs_;
	std::vector<OutputStage *> outputs_;
};

} // namespace tiling