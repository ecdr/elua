-- eLua platform interface - QEI

data_en = 
{
  -- Menu name
  menu_name = "QEI",

  -- Title
  title = "eLua platform interface - QEI",

  -- Overview
  overview = "This part of the platform interface groups functions related to the QEI interface(s) of the MCU.",

  -- Data structures, constants and types
  structures = 
  {
    { text = [[// eLua CAN ID types
enum
{
  ELUA_CAN_ID_STD = 0,
  ELUA_CAN_ID_EXT
};
]],
      name = "CAN ID types",
      desc = "Constants used to define whether the message ID is standard or extended.."
    }
  },

  -- Functions
  funcs = 
  {
    { sig = "int #platform_qei_exists#( unsigned id );",
      desc = [[Checks if the platform has the hardware QEI specified as argument. Implemented in %src/common.c%, it uses the $NUM_QEI$ macro that must be defined in the
  platform's $platform_conf.h$ file (see @arch_overview.html#platforms@here@ for details). For example:</p>
  ~#define NUM_QEI   1      $// The platform has one QEI interface$~<p> ]],
      args = "$id$ - QEI interface ID.",
      ret = "1 if the CAN interface exists, 0 otherwise"
    },

    { sig = "u32 #platform_qei_setup#( unsigned id );",
      desc = [[This function is used to initialize the QEI hardware.]],
      args = 
      {
        "$id$ - QEI interface ID.",
      },
      ret = ""
    },

    {  sig = "void #platform_qei_start#( unsigned id );",
       desc = "Start quadrature decoder.",
       args =
       {
          "$id$ - QEI interface ID.",
       },
    },

    {  sig = "void #platform_qei_stop#( unsigned id );",
       desc = "Stop quadrature decoder.",
       args =
       {
          "$id$ - QEI interface ID.",
       },
    },

    {  sig = "u32 #platform_qei_get_count#( unsigned id );",
       desc = "Read quadrature decoder position.",
       args =
       {
          "$id$ - QEI interface ID.",
       },
       ret = "Count (position) for quadrature decoder"
    },

    {  sig = "u32 #platform_qei_set_count#( unsigned id, u32 count );",
       desc = "Set quadrature decoder position.",
       args =
       {
          "$id$ - QEI interface ID.",
          "$count$ - QEI position value to set"
       },
       ret = "Count (position) for quadrature decoder, or error."
    },

    {  sig = "u32 #platform_qei_set_clock#( unsigned id, u32 freq );",
       desc = "Set quadrature clock for measuring speed.",
       args =
       {
          "$id$ - QEI interface ID.",
          "$freq$ - frequency for velocity measure, 0 for position only"
       },
       ret = "Quadrature decoder frequency actually set, or error."
    },

    {  sig = "u32 #platform_qei_get_dir#( unsigned id );",
       desc = "Read quadrature decoder direction.",
       args =
       {
          "$id$ - QEI interface ID.",
       },
       ret = "Direction for quadrature decoder, or error."
    },

    {  sig = "u32 #platform_qei_get_speed#( unsigned id );",
       desc = "Read quadrature decoder speed.",
       args =
       {
          "$id$ - QEI interface ID.",
       },
       ret = "Speed for quadrature decoder, or error."
    },


    {  sig = "int #platform_qei_set#( unsigned id, u32 attrib, u32 value );",
        desc = "Set QEI attribute.",
        args =
       {
          "$id$ - QEI interface ID.",
          "$attrib$ - QEI attribute ID (platform dependent).",
          "$value$ - value for given attribute",
       },
       ret = "PLATFORM_OK for success, or error"
    },

    {  sig = "int #platform_qei_get#( unsigned id, u32 attrib );",
        desc = "Receive CAN bus message.",
        args =
       {
          "$id$ - QEI interface ID.",
          "$attrib$ - QEI attribute ID (platform dependent).",
       },
       ret = "Value of given attribute"
    },
  }
}

