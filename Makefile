# Recepta kompilacji dla przykładowych programów. W zależności od tego
# z jakich bibliotek korzystają trzeba przy wywoływaniu kompilatora
# podawać różne opcje. Najpierw więc specyfikujemy rzeczy wspólne dla
# wszystkich programów (w sposób pozwalający zmodyfikować lub nadpisać
# je z linii komend przy uruchamianiu make)...

ifeq ($(origin CC),default)
CC := gcc
endif
ifeq ($(origin CXX),default)
CXX := g++
endif
CFLAGS := -std=c11 -pedantic -Wall $(CFLAGS)
CXXFLAGS := -std=c++14 -pedantic -Wall $(CXXFLAGS)
LDFLAGS := -Wl,--as-needed $(LDFLAGS)
LDLIBS := $(LDLIBS)

# ... a potem niektórym z nich uzupełniamy opcje. Znak procentu pełni tu
# rolę identyczną jak gwiazdka we wzorcach nazw plików.

gd% : LDLIBS += -lgd

# W pracowni nie mamy zainstalowanego pakietu libgtk-3-dev, ale może w domu
# ktoś będzie z nim eksperymentował. Jako kompromis Makefile pyta o opcje
# specyficzne dla tej biblioteki, ale z --silence-errors aby na uczelni nie
# pojawiały się niepokojące komunikaty błędów.
#
cairo01 : CFLAGS += $(shell pkg-config --silence-errors --cflags gtk+-3.0)
cairo01 : LDLIBS += $(shell pkg-config --silence-errors --libs gtk+-3.0)

cairo02 : CFLAGS += $(shell pkg-config --cflags cairo)
cairo02 : LDLIBS += $(shell pkg-config --libs cairo)

opencv% : CXXFLAGS += $(shell pkg-config --cflags opencv)
opencv% : LDLIBS += $(shell pkg-config --libs opencv)

osg% : CXXFLAGS += $(shell pkg-config --cflags openscenegraph)
osg% : LDLIBS += $(shell pkg-config --libs openscenegraph)

# Pierwszy cel tradycyjnie nazywa się "all". Kompiluje cały projekt (w naszym
# przypadku: wszystkie programy z wyjątkiem cairo01.c) jeśli make zostanie
# wywołane bez żadnego argumentu.

CSOURCES := $(wildcard *.c)
CXXSOURCES := $(wildcard *.cpp)
PROGRAMS := $(CSOURCES:.c=) $(CXXSOURCES:.cpp=)

all : $(filter-out cairo01,$(PROGRAMS))

# Jeden z programów dołącza dodatkowe pliki obiektowe powstałe z plików
# zawierających kod shaderów GLSL.

osg05 : $(patsubst %.glsl,%.o,$(wildcard phong_*.glsl))

%.o : %.glsl
	./txt2cstr $^ | $(CC) $(CFLAGS) -c -x c - -o $@

# Na koniec recepta na posprzątanie po sobie.

clean :
	rm -f $(PROGRAMS) *.o wynik*

.PHONY : all clean
