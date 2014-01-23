-- eLua reference manual - bitarray

data_en = 
{

  -- Title
  title = "eLua reference manual - bitarray module",

  -- Menu name
  menu_name = "bitarray"

  -- Overview
  overview = [[Compact, fixed size array of bits (or small integers).
  ]],

  -- Data structures, constants and types
  structures = 
  {
    { text = [[ ]],
      name = "", 
      desc = [[ ]]
    },
  },

  -- Functions
  funcs = 
  {
    { sig = "bitarray = #bitarray.new#( capacity, [element_size_bits], [fill] )",
--      array = bitarray.new( "string", [element_size_bits] ), or
--      array = bitarray.new( lua_array, [element_size_bits] )

      desc = [[ New bitarray ]],
      args = 
      {
        "$capacity$ - Number of elements in array",
        "$lua_array$ - Array to convert to bitarray,
        "$string$ - Convert string to bitarray,
        "$element_size_bits$ - Size of an element, in bits",
        "$fill$ - Initial value for elements",
      },
      ret = $bitarray$
    },

    { sig = "#array#[ index ]",
      desc = [[Return value of element in a bitarray.]],
      args = 
      {
        "$index$ - Number of element to access (starts at 1)",
      },
      ret = $number$
    },

    { sig = "#\##bitarray",		-- Probably needs escape
      desc = [[Return number of elements in a bitarray.]],
      ret = $number$
    },

    { sig = "#pairs#( bitarray )",
      desc = [[Return iterator over array indecies and elements.]],
      args = 
      {
        "$bitarray$ - array to traverse.",
      },
      ret = $iterator$
    },

    { sig = "string = #bitarray.tostring#( array, ["raw"|"seq"] )",
      desc = [[ Convert bitarray to string. ]],
      args = 
      {
        "$array$ - bitarray to convert",
      },
      ret = $string$ - String of bytes representing array
    },

-- Lua: table = bitarray.totable( array, ["raw"|"seq"] )
    { sig = "table = #bitarray.totable#( array, ["raw"|"seq"] )",
      desc = [[ Table containing contents of bitarray. ]],
      args = 
      {
        "$array$ - bitarray to convert",
      },
      ret = $table$
    },
  },


  -- Aux data
  auxdata = 
  {
    { title = "",
      desc = [[]]
    }
  }
}

data_pt = data_en
